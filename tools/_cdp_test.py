"""用无头 Chrome 真实点击「转向 / 偏航」按钮，抓取写往串口的字节。

页面脚本包在 IIFE 里，内部函数不对外暴露，所以这里不直接调函数，
而是注入一个假的 navigator.serial：点「连接」建立链路，再点按钮，
从 write() 拦截到的字节判断行为是否正确。串口日志轮询的 '?' 会被过滤。
"""
import json
import os
import shutil
import socket
import struct
import base64
import subprocess
import sys
import tempfile
import time
import urllib.request

CHROME = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
PAGE = r"D:\All_projects\STM32_Project\HAL\STM32F103C8T6\Robotic_arm\3.0\tools\serial_console.html"
PORT = 9333

FAKE_SERIAL = r"""
(() => {
  const enc = new TextEncoder();
  window.__writes = [];
  let pendingRead = null;

  const fakePort = {
    open: async () => {},
    close: async () => {},
    getInfo: () => ({ usbVendorId: 0x1a86, usbProductId: 0x7523 }),
    writable: {
      getWriter: () => ({
        write: async (data) => { window.__writes.push(new TextDecoder().decode(data)); return data.length; },
        releaseLock: () => {},
      }),
    },
    readable: {
      getReader: () => ({
        read: () => new Promise((res) => { pendingRead = res; }),
        cancel: async () => { if (pendingRead) { const r = pendingRead; pendingRead = null; r({ done: true }); } },
        releaseLock: () => {},
      }),
    },
    addEventListener: () => {},
    removeEventListener: () => {},
  };

  /* 从测试侧往页面"喂"一行串口数据，走的是真实的 onBytes -> 解析链路 */
  window.__pushLine = (s) => {
    if (pendingRead) { const r = pendingRead; pendingRead = null; r({ value: enc.encode(s), done: false }); }
  };

  Object.defineProperty(navigator, "serial", {
    configurable: true,
    value: {
      requestPort: async () => fakePort,
      getPorts: async () => [],
      addEventListener: () => {},
      removeEventListener: () => {},
    },
  });
})();
"""


class WS:
    def __init__(self, url):
        rest = url[5:]
        hostport, _, path = rest.partition("/")
        host, _, port = hostport.partition(":")
        self.s = socket.create_connection((host, int(port)), timeout=15)
        key = base64.b64encode(os.urandom(16)).decode()
        req = ("GET /" + path + " HTTP/1.1\r\nHost: " + hostport +
               "\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: " +
               key + "\r\nSec-WebSocket-Version: 13\r\n\r\n")
        self.s.sendall(req.encode())
        buf = b""
        while b"\r\n\r\n" not in buf:
            buf += self.s.recv(4096)
        if b"101" not in buf.split(b"\r\n")[0]:
            raise RuntimeError("WS 升级失败: " + buf[:120].decode(errors="replace"))

    def _read(self, n):
        out = b""
        while len(out) < n:
            chunk = self.s.recv(n - len(out))
            if not chunk:
                raise RuntimeError("连接断开")
            out += chunk
        return out

    def send(self, obj):
        data = json.dumps(obj).encode()
        mask = os.urandom(4)
        n = len(data)
        hdr = b"\x81"
        if n < 126:
            hdr += bytes([0x80 | n])
        elif n < 65536:
            hdr += bytes([0x80 | 126]) + struct.pack(">H", n)
        else:
            hdr += bytes([0x80 | 127]) + struct.pack(">Q", n)
        self.s.sendall(hdr + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))

    def recv(self):
        while True:
            b1, b2 = self._read(2)
            op = b1 & 0x0F
            n = b2 & 0x7F
            if n == 126:
                n = struct.unpack(">H", self._read(2))[0]
            elif n == 127:
                n = struct.unpack(">Q", self._read(8))[0]
            payload = self._read(n)
            if op == 1:
                return json.loads(payload.decode())
            if op == 8:
                raise RuntimeError("服务端关闭")


def call(ws, mid, method, params=None):
    ws.send({"id": mid, "method": method, "params": params or {}})
    while True:
        m = ws.recv()
        if m.get("id") == mid:
            return m


def ev(ws, mid, expr):
    r = call(ws, mid, "Runtime.evaluate",
             {"expression": expr, "awaitPromise": True, "returnByValue": True})
    res = r.get("result", {})
    if "exceptionDetails" in res:
        d = res["exceptionDetails"]
        return "<ERR> " + str(d.get("exception", {}).get("description", d.get("text")))
    return res.get("result", {}).get("value")


def main():
    if not os.path.exists(CHROME):
        print("跳过：没找到 Chrome")
        return 0

    prof = tempfile.mkdtemp(prefix="cdp-")
    proc = subprocess.Popen(
        [CHROME, "--headless=new", "--disable-gpu", "--no-first-run",
         "--no-default-browser-check", "--allow-file-access-from-files",
         f"--user-data-dir={prof}", f"--remote-debugging-port={PORT}", "about:blank"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    mid = [100]

    def n():
        mid[0] += 1
        return mid[0]

    ok = True

    def check(label, got, want):
        nonlocal ok
        good = got == want
        ok = ok and good
        print(("  OK   " if good else "  FAIL ") + label +
              f"  got={got!r}" + ("" if good else f"  want={want!r}"))

    try:
        for _ in range(80):
            try:
                urllib.request.urlopen(f"http://127.0.0.1:{PORT}/json/version", timeout=1).read()
                break
            except Exception:
                time.sleep(0.25)
        else:
            print("FAIL chrome 没起来")
            return 1

        url = "file:///" + PAGE.replace("\\", "/")
        req = urllib.request.Request(f"http://127.0.0.1:{PORT}/json/new?{url}", method="PUT")
        tab = json.loads(urllib.request.urlopen(req, timeout=10).read())
        ws = WS(tab["webSocketDebuggerUrl"])
        call(ws, n(), "Page.enable")
        call(ws, n(), "Runtime.enable")

        # 注入假串口，然后重新加载，确保页面脚本启动时 navigator.serial 已存在
        call(ws, n(), "Page.addScriptToEvaluateOnNewDocument", {"source": FAKE_SERIAL})
        call(ws, n(), "Page.reload", {"ignoreCache": True})
        time.sleep(1.5)

        print("== 1. 页面加载")
        check("标题", ev(ws, n(), "document.title"), "机械臂串口控制台")
        check("注入 serial", ev(ws, n(), "'serial' in navigator"), True)
        check("按钮初始禁用", ev(ws, n(), "document.getElementById('btnYawPos').disabled"), True)
        check("按钮初始文案", ev(ws, n(), "document.getElementById('btnYawPos').textContent"), "+15°")

        print("== 2. 点「连接」，建立假串口链路")
        ev(ws, n(), "document.getElementById('btnConn').click()")
        time.sleep(0.6)
        state = ev(ws, n(), "document.getElementById('connState').textContent")
        print("   状态文本:", state)
        check("已连接", bool(state) and state.startswith("已连接"), True)
        check("按钮已启用", ev(ws, n(), "document.getElementById('btnYawPos').disabled"), False)

        def take():
            raw = ev(ws, n(), "window.__writes.splice(0)")
            return [w for w in (raw or []) if w.strip() != "?"]

        take()  # 丢掉连接时的残留（含轮询 '?'）

        print("== 3. 点 +15 两次（验证乐观更新，不会发出两条一样的命令）")
        ev(ws, n(), "document.getElementById('btnYawPos').click()")
        time.sleep(0.35)
        check("第一次", take(), ["Y 15\r\n"])
        ev(ws, n(), "document.getElementById('btnYawPos').click()")
        time.sleep(0.35)
        check("第二次累加", take(), ["Y 30\r\n"])

        print("== 4. 步长改 5 后点 −5")
        ev(ws, n(), "const e=document.getElementById('inYawStep'); e.value=5;"
                   "e.dispatchEvent(new Event('input'));")
        check("负按钮文案", ev(ws, n(), "document.getElementById('btnYawNeg').textContent"), "−5°")
        ev(ws, n(), "document.getElementById('btnYawNeg').click()")
        time.sleep(0.35)
        check("发出", take(), ["Y 25\r\n"])

        print("== 5. 限位夹取：起点 25，步长 88 → 应夹到 88")
        ev(ws, n(), "window.__writes.length=0;"
                   "const e2=document.getElementById('inYawStep'); e2.value=88;"
                   "e2.dispatchEvent(new Event('input'));")
        check("正按钮文案", ev(ws, n(), "document.getElementById('btnYawPos').textContent"), "+88°")
        ev(ws, n(), "document.getElementById('btnYawPos').click()")
        time.sleep(0.35)
        check("被夹到 88", take(), ["Y 88\r\n"])
        check("提示已到限位", "限位" in (ev(ws, n(),
              "document.getElementById('log').textContent") or ""), True)

        print("== 6. 回正")
        ev(ws, n(), "document.getElementById('btnYawZero').click()")
        time.sleep(0.35)
        check("发出", take(), ["Y 0\r\n"])

        print("== 7. 固件回报解析：底座角 105 → 偏航 −15")
        # 注意符号：底座舵机转向与模型约定相反（model.h 的 SERVO_DIR_W = -1，
        # 网页 WAIST.dir 同步为 -1），所以 waist 105 对应的逻辑偏航是 −15 而不是 +15。
        # 改成 DIR_W = +1 时这条断言要跟着改回 +15。
        ev(ws, n(), "window.__pushLine('S 280 100 72 105 120 70 42 0 60 0\\r\\n')")
        time.sleep(0.4)
        hint = ev(ws, n(), "document.getElementById('yawHint').textContent")
        print("   hint:", hint)
        check("回报含 -15", "-15" in (hint or ""), True)

        print("== 8. 让回报把乐观值纠正回来：实在 90 → 偏航 0，步长恢复 15")
        ev(ws, n(), "const e3=document.getElementById('inYawStep'); e3.value=15;"
                   "e3.dispatchEvent(new Event('input'));"
                   "window.__pushLine('S 280 100 0 90 120 70 42 0 60 0\\r\\n')")
        time.sleep(0.4)
        hint8 = ev(ws, n(), "document.getElementById('yawHint').textContent")
        print("   hint:", hint8)
        check("回报后偏航归零", "偏航 0°" in (hint8 or ""), True)
        ev(ws, n(), "document.getElementById('btnYawPos').click()")
        time.sleep(0.35)
        check("基于回报值发出 +15", take(), ["Y 15\r\n"])

        print("== 9. 页面异常")
        errs = ev(ws, n(), "window.__pageErrs || []")
        print("   控制台错误:", errs if errs else "无")

        print("\n结果:", "全部通过" if ok else "有失败项")
        return 0 if ok else 2
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            proc.kill()
        shutil.rmtree(prof, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
