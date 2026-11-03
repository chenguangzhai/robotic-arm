"""点表缓存的端到端验证：断联不丢、刷新不丢、断电后能推回。

页面脚本在 IIFE 里，内部函数不暴露，所以全程"真点击 + 假串口"：
  - 起一个本地 http 服务（file:// 下 localStorage 的可见性各浏览器不一致，
    用 http 才能稳定验证持久化）
  - 注入假 navigator.serial，读方向由 __pushRaw 喂，写方向带自动应答，
    用一个 JS 侧的小"设备模型"充当固件（C / A / L 三条命令的真语义）
  - 断言写出的字节序列和面板文案
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _cdp_test import WS, call  # noqa: E402

CHROME = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
TOOLS = os.path.dirname(os.path.abspath(__file__))
PORT_HTTP = 8931
PORT_CDP = 9337

FAKE = r"""
(() => {
  const enc = new TextEncoder();
  const q = [];
  let pendingRead = null;

  function deliver() {
    while (pendingRead && q.length) {
      const r = pendingRead; pendingRead = null;
      r({ value: q.shift(), done: false });
    }
  }
  window.__writes = [];
  window.__dev = { points: [] };
  window.__autoReply = null;

  const fakePort = {
    open: async () => {},
    close: async () => {},
    getInfo: () => ({ usbVendorId: 0x1a86, usbProductId: 0x7523 }),
    writable: {
      getWriter: () => ({
        write: async (data) => {
          const t = new TextDecoder().decode(data);
          window.__writes.push(t);
          if (window.__autoReply) {
            const out = window.__autoReply(t);
            if (out) { q.push(enc.encode(out)); deliver(); }
          }
          return data.length;
        },
        releaseLock: () => {},
      }),
    },
    readable: {
      getReader: () => ({
        read: () => new Promise((res) => { pendingRead = res; deliver(); }),
        cancel: async () => { if (pendingRead) { const r = pendingRead; pendingRead = null; r({ done: true }); } },
        releaseLock: () => {},
      }),
    },
    addEventListener: () => {},
    removeEventListener: () => {},
  };

  window.__pushRaw = (s) => { q.push(enc.encode(s)); deliver(); };

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

# 固件语义的极简模型：C 清空 / A 追加 / L 列出
DEVICE = r"""
(() => {
  window.__autoReply = (raw) => {
    const s = raw.trim();
    if (s === "C") { window.__dev.points = []; return "OK\r\n"; }
    if (/^A\s/.test(s)) {
      const p = s.split(/\s+/).slice(1).map(Number);
      if (p.length < 6) { return "ERR:ARG\r\n"; }
      window.__dev.points.push({ a: p.slice(0, 6), dwell: p.length > 6 ? p[6] : 500 });
      return "OK " + window.__dev.points.length + "\r\n";
    }
    if (s === "L") {
      let out = "";
      window.__dev.points.forEach((p, k) => {
        out += "L " + (k + 1) + " " + p.a.join(" ") + " " + p.dwell + "\r\n";
      });
      return out + "L END " + window.__dev.points.length + "\r\n";
    }
    if (s === "?") { return ""; }        /* 不回报状态，避免干扰断言 */
    if (s === "P" || /^P\s/.test(s)) { return "OK " + window.__dev.points.length + "\r\n"; }
    return "";
  };
})();
"""


def main():
    if not os.path.exists(CHROME):
        print("跳过：没找到 Chrome")
        return 0

    handler = partial(SimpleHTTPRequestHandler, directory=TOOLS)
    srv = ThreadingHTTPServer(("127.0.0.1", PORT_HTTP), handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    prof = tempfile.mkdtemp(prefix="cdp-pts-")
    proc = subprocess.Popen(
        [CHROME, "--headless=new", "--disable-gpu", "--no-first-run",
         "--no-default-browser-check",
         f"--user-data-dir={prof}", f"--remote-debugging-port={PORT_CDP}", "about:blank"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    mid = [200]
    ok = True

    def n():
        mid[0] += 1
        return mid[0]

    def check(label, got, want):
        nonlocal ok
        good = got == want
        ok = ok and good
        print(("  OK   " if good else "  FAIL ") + label +
              f"  got={got!r}" + ("" if good else f"  want={want!r}"))

    def ev(expr):
        r = call(ws, n(), "Runtime.evaluate",
                 {"expression": expr, "awaitPromise": True, "returnByValue": True})
        res = r.get("result", {})
        if "exceptionDetails" in res:
            d = res["exceptionDetails"]
            return "<ERR> " + str(d.get("exception", {}).get("description", d.get("text")))
        return res.get("result", {}).get("value")

    try:
        for _ in range(80):
            try:
                urllib.request.urlopen(f"http://127.0.0.1:{PORT_CDP}/json/version", timeout=1).read()
                break
            except Exception:
                time.sleep(0.25)
        else:
            print("FAIL chrome 没起来")
            return 1

        url = f"http://127.0.0.1:{PORT_HTTP}/serial_console.html"
        req = urllib.request.Request(f"http://127.0.0.1:{PORT_CDP}/json/new?{url}", method="PUT")
        tab = json.loads(urllib.request.urlopen(req, timeout=10).read())
        ws = WS(tab["webSocketDebuggerUrl"])
        call(ws, n(), "Page.enable")
        call(ws, n(), "Runtime.enable")
        call(ws, n(), "Page.addScriptToEvaluateOnNewDocument", {"source": FAKE})
        call(ws, n(), "Page.addScriptToEvaluateOnNewDocument", {"source": DEVICE})
        call(ws, n(), "Page.reload", {"ignoreCache": True})
        time.sleep(1.2)
        ev("window.confirm = () => true;")

        rows = "document.querySelectorAll('#ptList .prow').length"
        hint = "document.getElementById('ptHint').textContent"

        print("== 1. 首次打开：缓存为空")
        check("点行数", ev(rows), 0)
        check("localStorage 可写", ev("(()=>{try{localStorage.setItem('t','1');"
                                     "localStorage.removeItem('t');return true}catch(e){return String(e)}})()"), True)

        print("== 2. 连接后设备有 2 个点，以设备为准")
        ev("window.__dev.points = [{a:[120,70,45,30,0,60],dwell:500},"
           "{a:[110,80,50,40,10,60],dwell:700}];")
        ev("document.getElementById('btnConn').click()")
        time.sleep(0.7)
        check("点行数", ev(rows), 2)
        h = ev(hint)
        print("   hint:", h)
        check("标为来自设备", "来自设备" in (h or ""), True)

        print("== 3. 断开：列表不清空（这是核心需求）")
        ev("document.getElementById('btnConn').click()")
        time.sleep(0.5)
        check("断开后点行数", ev(rows), 2)
        h = ev(hint)
        print("   hint:", h)
        check("标为网页缓存", "网页缓存" in (h or ""), True)

        print("== 4. 刷新页面：从 localStorage 恢复")
        call(ws, n(), "Page.reload", {"ignoreCache": True})
        time.sleep(1.2)
        ev("window.confirm = () => true;")
        check("刷新后点行数", ev(rows), 2)
        h = ev(hint)
        print("   hint:", h)
        check("标为网页缓存", "网页缓存" in (h or ""), True)

        print("== 5. 重连但设备已断电丢点（L END 0）：缓存不被覆盖")
        ev("window.__dev.points = [];")
        ev("document.getElementById('btnConn').click()")
        time.sleep(0.7)
        check("点行数保持", ev(rows), 2)
        check("仍标为网页缓存", "网页缓存" in (ev(hint) or ""), True)

        print("== 6. 此时点「试放」应被拦住，不发 P")
        ev("window.__writes.length = 0;")
        ev("document.getElementById('btnPtPlayOnce').click()")
        time.sleep(0.4)
        sent = [w for w in (ev("window.__writes.splice(0)") or []) if w.strip() not in ("", "?")]
        check("没有发出 P", [w for w in sent if w.strip().startswith("P")], [])

        print("== 7. 「推回单片机」：C 清空 + 逐条 A")
        ev("window.__writes.length = 0;")
        ev("document.getElementById('btnPtPush').click()")
        time.sleep(1.2)
        sent = [w.strip() for w in (ev("window.__writes.splice(0)") or [])
                if w.strip() not in ("", "?")]
        print("   发出:", sent)
        check("首条是清空", sent[0] if sent else None, "C")
        check("第二条是第 1 点", sent[1] if len(sent) > 1 else None, "A 120 70 45 30 0 60 500")
        check("第三条是第 2 点", sent[2] if len(sent) > 2 else None, "A 110 80 50 40 10 60 700")
        check("设备点数", ev("window.__dev.points.length"), 2)
        check("设备第 1 点角度", ev("JSON.stringify(window.__dev.points[0].a)"), "[120,70,45,30,0,60]")
        h = ev(hint)
        print("   hint:", h)
        check("推回后标为来自设备", "来自设备" in (h or ""), True)

        print("== 8. 导入 JSON：只进网页缓存")
        ev("window.__writes.length = 0;"
           "const f = new File([JSON.stringify({points:[{a:[10,20,30,40,50,60],dwell:300},"
           "{a:[11,21,31,41,51,61],dwell:400}]})], 't.json', {type:'application/json'});"
           "const dt = new DataTransfer(); dt.items.add(f);"
           "const inp = document.getElementById('filePtImport'); inp.files = dt.files;"
           "inp.dispatchEvent(new Event('change'));")
        time.sleep(0.6)
        check("导入后点行数", ev(rows), 2)
        check("导入不动设备", ev("window.__writes.filter(w=>w.trim()!=='?').length"), 0)
        check("标为网页缓存", "网页缓存" in (ev(hint) or ""), True)
        check("P 按钮仍被拦（缓存态）", ev(
            "(()=>{window.__writes.length=0;document.getElementById('btnPtPlayOnce').click();"
            "return window.__writes.filter(w=>w.trim().startsWith('P')).length})()"), 0)

        print("== 9. 清空：连缓存一起清，不留幻影")
        ev("document.getElementById('btnPtClear').click()")
        time.sleep(0.6)
        check("点行数", ev(rows), 0)
        call(ws, n(), "Page.reload", {"ignoreCache": True})
        time.sleep(1.2)
        check("刷新后仍为空（key 已删）", ev(rows), 0)

        print("== 10. 页面异常")
        errs = ev("window.__pageErrs || []")
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
        srv.shutdown()


if __name__ == "__main__":
    sys.exit(main())
