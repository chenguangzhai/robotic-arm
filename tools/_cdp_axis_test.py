"""用无头 Chrome 验证「目标坐标」第三个框的 Z / 偏航角 开关。

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
PAGE = r"D:\All_projects\STM32_Project\HAL\STM32F103C8T6\Robotic_arm\tools\serial_console.html"
PORT = 9341

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

    def near(label, got, want, tol):
        nonlocal ok
        good = got is not None and abs(got - want) <= tol
        ok = ok and good
        print(("  OK   " if good else "  FAIL ") + label +
              f"  got={got!r}" + ("" if good else f"  want≈{want} (±{tol})"))

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

        call(ws, n(), "Page.addScriptToEvaluateOnNewDocument", {"source": FAKE_SERIAL})
        call(ws, n(), "Page.addScriptToEvaluateOnNewDocument", {
            "source": "window.__pageErrs=[];"
                      "addEventListener('error',e=>window.__pageErrs.push(String(e.message)));"})
        call(ws, n(), "Page.reload", {"ignoreCache": True})
        time.sleep(1.5)

        print("== 1. 初始状态")
        check("标题", ev(ws, n(), "document.title"), "机械臂串口控制台")
        check("第三个框标签", ev(ws, n(), "document.getElementById('lbThird').textContent"), "Z 横向")
        check("开关默认在 Z", ev(ws, n(), "document.getElementById('btnAxisZ').className"), "on")
        check("偏航提示默认隐藏",
              ev(ws, n(), "getComputedStyle(document.getElementById('thirdHint')).display"), "none")

        print("== 2. 连接假串口")
        ev(ws, n(), "document.getElementById('btnConn').click()")
        time.sleep(0.6)

        def take():
            raw = ev(ws, n(), "window.__writes.splice(0)")
            return [w for w in (raw or []) if w.strip() != "?"]

        def fill(x, y, third):
            ev(ws, n(), "(()=>{const s=(id,v)=>{const e=document.getElementById(id);"
                       "e.value=v; e.dispatchEvent(new Event('input'));};"
                       f"s('inX','{x}'); s('inY','{y}'); s('inZ','{third}');}})()")

        def send():
            ev(ws, n(), "window.__writes.length=0;")
            ev(ws, n(), "document.getElementById('btnSend').click()")
            time.sleep(0.35)
            return take()

        take()  # 丢掉连接时的残留

        print("== 3. Z 模式：原样发 M x y z")
        fill(0.28, 0.10, 0.00)
        check("发出", send(), ["M 0.2800 0.1000 0.0000\r\n"])

        print("== 4. 切到偏航角：同一目标点被换算成 0.0°")
        ev(ws, n(), "document.getElementById('btnAxisYaw').click()")
        time.sleep(0.2)
        check("标签改成偏航", ev(ws, n(), "document.getElementById('lbThird').textContent"), "偏航 θ (°)")
        check("开关高亮", ev(ws, n(), "document.getElementById('btnAxisYaw').className"), "on")
        check("值换算为 0（点没变）", ev(ws, n(), "document.getElementById('inZ').value"), "0.0")
        hint = ev(ws, n(), "document.getElementById('thirdHint').textContent") or ""
        print("   hint:", hint)
        check("提示可见", ev(ws, n(),
              "getComputedStyle(document.getElementById('thirdHint')).display") != "none", True)
        check("提示含换算", "z =" in hint, True)

        print("== 5. 偏航 30° → z = 0.28·tan30 = 0.1617")
        ev(ws, n(), "(()=>{const e=document.getElementById('inZ'); e.value='30';"
                   "e.dispatchEvent(new Event('input'));})()")
        check("预览命令", ev(ws, n(), "document.getElementById('cmdPreview').textContent")[:23],
              "M 0.2800 0.1000 0.1617")
        check("发出", send(), ["M 0.2800 0.1000 0.1617\r\n"])
        hint5 = ev(ws, n(), "document.getElementById('thirdHint').textContent") or ""
        print("   hint:", hint5)
        check("提示写了半径", "半径" in hint5, True)

        print("== 6. 切回 Z：角度还原成 0.162")
        ev(ws, n(), "document.getElementById('btnAxisZ').click()")
        time.sleep(0.2)
        check("标签", ev(ws, n(), "document.getElementById('lbThird').textContent"), "Z 横向")
        check("z 还原", ev(ws, n(), "document.getElementById('inZ').value"), "0.162")
        check("提示又隐藏",
              ev(ws, n(), "getComputedStyle(document.getElementById('thirdHint')).display"), "none")

        print("== 7. 反向：z=0.10 切偏航 → atan2(0.10,0.28)=19.7°")
        fill(0.28, 0.10, 0.10)
        ev(ws, n(), "document.getElementById('btnAxisYaw').click()")
        time.sleep(0.2)
        check("角度", ev(ws, n(), "document.getElementById('inZ').value"), "19.7")

        print("== 8. 越界：偏航 100° 应当拒绝，不发任何字节")
        ev(ws, n(), "(()=>{const e=document.getElementById('inZ'); e.value='100';"
                   "e.dispatchEvent(new Event('input'));})()")
        log_before = ev(ws, n(), "document.getElementById('log').textContent") or ""
        check("不发字节", send(), [])
        log8 = ev(ws, n(), "document.getElementById('log').textContent") or ""
        check("日志提示限位", ("±88" in log8[len(log_before):]), True)

        print("== 9. x=0 时无法换算，应拒绝并说明")
        ev(ws, n(), "(()=>{const e=document.getElementById('inZ'); e.value='30';"
                   "e.dispatchEvent(new Event('input'));"
                   "const x=document.getElementById('inX'); x.value='0';"
                   "x.dispatchEvent(new Event('input'));})()")
        log_before9 = ev(ws, n(), "document.getElementById('log').textContent") or ""
        check("不发字节", send(), [])
        log9 = (ev(ws, n(), "document.getElementById('log').textContent") or "")[len(log_before9):]
        print("   新增日志:", log9.strip().splitlines()[-1:] )
        check("日志提示 X 要大于 0", "大于 0" in log9, True)

        print("== 10. 切毫米：x/y 换算，角度不动")
        ev(ws, n(), "(()=>{const x=document.getElementById('inX'); x.value='0.28';"
                   "x.dispatchEvent(new Event('input'));"
                   "const u=document.getElementById('unit'); u.value='mm';"
                   "u.dispatchEvent(new Event('change'));})()")
        check("x 换成毫米", ev(ws, n(), "document.getElementById('inX').value"), "280")
        check("角度保持不变", ev(ws, n(), "document.getElementById('inZ').value"), "30")
        check("发出（单位系数生效）", send(), ["M 0.2800 0.1000 0.1617\r\n"])
        # 换回米，别把后面的断言带偏
        ev(ws, n(), "(()=>{const u=document.getElementById('unit'); u.value='m';"
                   "u.dispatchEvent(new Event('change'));})()")

        print("== 11. 3D 点选在两种模式下指向同一个点")
        ev(ws, n(), "document.getElementById('btnView3D').click()")
        time.sleep(0.3)
        ev(ws, n(), "document.getElementById('btnAxisZ').click()")
        time.sleep(0.2)
        fill(0.28, 0.10, 0.00)

        rect = ev(ws, n(), "(()=>{const r=document.getElementById('armCanvas')"
                           ".getBoundingClientRect();"
                           "return JSON.stringify([r.left, r.top, r.width, r.height]);})()")
        rect = json.loads(rect)
        cx = rect[0] + rect[2] * 0.55
        cy = rect[1] + rect[3] * 0.62
        for t in ("mousePressed", "mouseReleased"):
            call(ws, n(), "Input.dispatchMouseEvent",
                 {"type": t, "x": cx, "y": cy, "button": "left", "clickCount": 1})
        time.sleep(0.3)

        px = float(ev(ws, n(), "document.getElementById('inX').value"))
        pz = float(ev(ws, n(), "document.getElementById('inZ').value"))
        print(f"   点选结果 x={px} z={pz}")
        check("点选填了 x", px > 0, True)
        ev(ws, n(), "document.getElementById('btnAxisYaw').click()")
        time.sleep(0.2)
        deg = float(ev(ws, n(), "document.getElementById('inZ').value"))
        import math
        near("角度 = atan2(z,x)", deg, math.degrees(math.atan2(pz, px)), 0.15)
        ev(ws, n(), "document.getElementById('btnAxisZ').click()")
        time.sleep(0.2)
        z_back = float(ev(ws, n(), "document.getElementById('inZ').value"))
        near("换回来还是同一个 z", z_back, pz, 0.002)

        print("== 12. 页面异常")
        errs = ev(ws, n(), "window.__pageErrs || []")
        print("   控制台错误:", errs if errs else "无")
        check("无脚本异常", errs, [])

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
