"""验证「演示模式」：不接硬件，内置模拟设备能否让整个界面正常跑起来。

页面脚本包在 IIFE 里、内部函数不对外暴露，所以这里全程走真实交互：
点按钮、往发送框里填指令、读 DOM。断言的都是用户能看到的东西
（状态灯、坐标、夹爪状态、日志、点表），而不是内部变量。

跑法：python tools/_cdp_demo_test.py
"""
import json
import os
import socket
import struct
import subprocess
import tempfile
import time
import urllib.request

CHROME = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PAGE = os.path.join(ROOT, "tools", "serial_console.html")
PORT = 9335


class WS:
    def __init__(self, url):
        path = url.split("://", 1)[1]
        hostport, p = path.split("/", 1)
        host, port = hostport.split(":")
        self.s = socket.create_connection((host, int(port)), timeout=20)
        key = "dGhlIHNhbXBsZSBub25jZQ=="
        req = (f"GET /{p} HTTP/1.1\r\nHost: {hostport}\r\nUpgrade: websocket\r\n"
               f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
               f"Sec-WebSocket-Version: 13\r\n\r\n")
        self.s.sendall(req.encode())
        buf = b""
        while b"\r\n\r\n" not in buf:
            buf += self.s.recv(4096)

    def _read(self, n):
        out = b""
        while len(out) < n:
            c = self.s.recv(n - len(out))
            if not c:
                raise RuntimeError("连接断了")
            out += c
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

    prof = tempfile.mkdtemp(prefix="cdp-demo-")
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
        time.sleep(1.2)

        print("== 1. 初始状态（未连接）")
        check("未连接", ev(ws, n(), "document.getElementById('connState').textContent"), "未连接")
        check("发送按钮禁用", ev(ws, n(), "document.getElementById('btnSend').disabled"), True)
        check("演示按钮可用", ev(ws, n(), "document.getElementById('btnDemo').disabled"), False)

        print("== 2. 点「演示模式」")
        ev(ws, n(), "document.getElementById('btnDemo').click()")
        time.sleep(0.5)
        state = ev(ws, n(), "document.getElementById('connState').textContent")
        print("   状态文本:", state)
        check("进入演示", "演示模式" in (state or ""), True)
        check("演示按钮变退出",
              ev(ws, n(), "document.getElementById('btnDemo').textContent"), "退出演示")
        check("连接按钮禁用",
              ev(ws, n(), "document.getElementById('btnConn').disabled"), True)
        check("发送按钮启用",
              ev(ws, n(), "document.getElementById('btnSend').disabled"), False)

        print("== 3. 设备应答：L 点表应被读到 4 个点")
        time.sleep(0.6)
        log = ev(ws, n(), "document.getElementById('log').textContent") or ""
        check("日志有 READY", "READY 3.0" in log, True)
        check("日志有 L END 4", "L END 4" in log, True)
        hint = ev(ws, n(), "document.getElementById('ptHint').textContent") or ""
        print("   点表提示:", hint)
        check("点表非空", "4" in hint, True)

        print("== 4. 自动循环回放：应出现 PLAY 事件且姿态在变")
        time.sleep(1.6)
        log = ev(ws, n(), "document.getElementById('log').textContent") or ""
        check("有 PLAY 1/4", "PLAY 1/4" in log, True)
        tip_a = ev(ws, n(), "document.getElementById('vTip').textContent")
        time.sleep(1.4)
        tip_b = ev(ws, n(), "document.getElementById('vTip').textContent")
        print("   姿态:", tip_a, "→", tip_b)
        check("姿态在变化", tip_a != tip_b and "—" not in (tip_a or ""), True)

        print("== 5. 停止回放（X 应停在当前姿态，不是走完这一段）")
        ev(ws, n(), "document.getElementById('btnPtStop').click()")
        time.sleep(0.5)
        tip_s1 = ev(ws, n(), "document.getElementById('vTip').textContent")
        time.sleep(1.0)
        tip_s2 = ev(ws, n(), "document.getElementById('vTip').textContent")
        print("   停止后姿态:", tip_s1, "→", tip_s2)
        check("姿态冻结", tip_s1 == tip_s2, True)

        print("== 6. 发 M（逆解 + 直线运动）")
        ev(ws, n(), "(function(){const m=document.getElementById('manual');"
                   "m.value='M 0.30 0.10 0';document.getElementById('btnManual').click();})()")
        time.sleep(2.8)
        pos = ev(ws, n(), "document.getElementById('stPos').textContent") or ""
        err = ev(ws, n(), "document.getElementById('vErr').textContent") or ""
        pill = ev(ws, n(), "document.getElementById('resultPill').textContent") or ""
        print("   坐标:", pos, "| 偏差:", err, "| 状态:", pill)
        check("到位", "已到位" in pill, True)
        m = pos.replace("m", "").replace(" ", "").split(",")
        check("x 接近 0.30", abs(float(m[0]) - 0.30) < 0.01, True)
        check("y 接近 0.10", abs(float(m[1]) - 0.10) < 0.01, True)

        print("== 7. 背后方向应被拒（ERR:OUT）")
        tip_err0 = ev(ws, n(), "document.getElementById('vTip').textContent")
        ev(ws, n(), "(function(){const m=document.getElementById('manual');"
                   "m.value='M -0.28 0.10 0';document.getElementById('btnManual').click();})()")
        time.sleep(1.0)
        pill = ev(ws, n(), "document.getElementById('resultPill').textContent") or ""
        print("   状态灯:", pill)
        check("报超出工作空间", "超出工作空间" in pill, True)
        check("被拒后没动", ev(ws, n(), "document.getElementById('vTip').textContent"),
              tip_err0)

        print("== 8. 夹爪 G 1 → 60° 收紧")
        ev(ws, n(), "(function(){const m=document.getElementById('manual');"
                   "m.value='G 1';document.getElementById('btnManual').click();})()")
        time.sleep(0.6)
        g = ev(ws, n(), "document.getElementById('stG').textContent") or ""
        print("   夹爪:", g)
        check("夹爪收紧", "60" in g and "收紧" in g, True)

        print("== 9. 手动发 C 清空设备点表（面板故意保留网页缓存，见 8.6）")
        ev(ws, n(), "(function(){const m=document.getElementById('manual');"
                   "m.value='C';document.getElementById('btnManual').click();})()")
        time.sleep(0.6)
        ev(ws, n(), "(function(){const m=document.getElementById('manual');"
                   "m.value='L';document.getElementById('btnManual').click();})()")
        time.sleep(0.8)
        log = ev(ws, n(), "document.getElementById('log').textContent") or ""
        hint = ev(ws, n(), "document.getElementById('ptHint').textContent") or ""
        print("   点表提示:", hint)
        check("设备回 OK 0", "OK 0" in log, True)
        check("设备点表为空（L END 0）", "L END 0" in log, True)
        check("面板保留缓存并标注", "缓存" in hint, True)

        print("== 10. 退出演示，界面回到未连接")
        ev(ws, n(), "document.getElementById('btnDemo').click()")
        time.sleep(0.5)
        check("已退出", ev(ws, n(), "document.getElementById('connState').textContent"), "未连接")
        check("发送按钮重新禁用",
              ev(ws, n(), "document.getElementById('btnSend').disabled"), True)

        print("== 11. ?demo=1 自动进入演示")
        req = urllib.request.Request(
            f"http://127.0.0.1:{PORT}/json/new?{url}?demo=1", method="PUT")
        tab2 = json.loads(urllib.request.urlopen(req, timeout=10).read())
        ws2 = WS(tab2["webSocketDebuggerUrl"])
        call(ws2, n(), "Page.enable")
        call(ws2, n(), "Runtime.enable")
        time.sleep(1.8)
        state2 = ev(ws2, n(), "document.getElementById('connState').textContent") or ""
        print("   状态文本:", state2)
        check("自动演示", "演示模式" in state2, True)
    finally:
        try:
            proc.terminate()
        except Exception:
            pass

    print("\n" + ("全部通过" if ok else "有失败项"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
