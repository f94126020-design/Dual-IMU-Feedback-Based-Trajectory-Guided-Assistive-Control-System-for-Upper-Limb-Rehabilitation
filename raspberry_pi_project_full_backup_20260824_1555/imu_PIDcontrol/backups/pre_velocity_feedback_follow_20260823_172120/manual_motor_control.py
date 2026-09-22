#!/usr/bin/env python3
"""Safe interactive USB-Serial motor test console for the ESP32 node.

This tool is intentionally independent from the Flask GUI.  Stop the
``imu-control.service`` before running it so that only one process owns the
ESP32 serial port.  Every PWM command is time-bounded and ends with STOP.
"""

import argparse
import glob
import shlex
import subprocess
import sys
import threading
import time

try:
    # Name used by the live Raspberry Pi project.
    from esp32motor import ESP32MotorBridge
except ImportError:
    # Compatibility with the Windows backup name.
    from esp32_motor_bridge import ESP32MotorBridge


TRICEPS_RELEASE_RATIO = 0.5
BICEPS_RELEASE_RATIO = 0.8


def service_is_active():
    try:
        result = subprocess.run(
            ["systemctl", "is-active", "--quiet", "imu-control.service"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return result.returncode == 0
    except FileNotFoundError:
        return False


def candidate_ports(requested_port=None):
    if requested_port:
        return [requested_port]
    return sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))


def connect_bridge(requested_port=None):
    ports = candidate_ports(requested_port)
    if not ports:
        raise RuntimeError("找不到 /dev/ttyUSB* 或 /dev/ttyACM*，請確認 ESP32 USB 已接上。")

    errors = []
    for port in ports:
        bridge = ESP32MotorBridge(port=port, baudrate=115200, timeout=0.05, pwm_limit=255)
        try:
            print(f"正在連線 {port} ...")
            bridge.connect()
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and not bridge.last_line:
                time.sleep(0.05)
            if not bridge.last_line:
                raise RuntimeError("裝置沒有回傳 ESP32 狀態")
            bridge.stop()
            time.sleep(0.1)
            print(f"已連線：{port}｜ESP32 回覆：{bridge.last_line}")
            return bridge
        except Exception as exc:
            errors.append(f"{port}: {exc}")
            try:
                bridge.close()
            except Exception:
                pass
    raise RuntimeError("ESP32 連線失敗：" + "；".join(errors))


def parse_pwm(value):
    pwm = int(value)
    if not -255 <= pwm <= 255:
        raise ValueError("PWM 必須介於 -255 到 255。")
    return pwm


def parse_duration(value, maximum):
    duration = float(value)
    if not 0.05 <= duration <= maximum:
        raise ValueError(f"持續時間必須介於 0.05 到 {maximum:g} 秒。")
    return duration


def paired_pwm(
    elbow_pwm,
    deltoid_pwm=0,
    triceps_release_ratio=TRICEPS_RELEASE_RATIO,
    biceps_release_ratio=BICEPS_RELEASE_RATIO,
):
    """Build the calibrated antagonist output for the two elbow tendons."""
    elbow = parse_pwm(elbow_pwm)
    deltoid = parse_pwm(deltoid_pwm)
    if elbow > 0:
        # Biceps winds while triceps pays out. The triceps spool releases more
        # cable per equal PWM, so reduce only this payout direction.
        biceps = elbow
        triceps = -int(round(elbow * float(triceps_release_ratio)))
    else:
        # Triceps winds while biceps pays out. Keep a separately adjustable
        # ratio because motor/spool friction is not symmetric.
        biceps = -int(round(abs(elbow) * float(biceps_release_ratio)))
        triceps = -elbow
    return biceps, triceps, deltoid


def has_valid_antagonist_pair(values):
    if len(values) != 3:
        return False
    biceps, triceps, _ = values
    return (biceps == 0 and triceps == 0) or (biceps * triceps < 0)


def print_help(max_duration):
    print(
        """
指令：
  elbow <PWM> <秒數>               二頭／三頭校正比例同步測試
  shoulder <PWM> <秒數>            三角肌單獨測試，肘肌維持 0
  all <肘PWM> <肩PWM> <秒數>       肘肌校正比例＋三角肌同步輸出
  status                            查看 ESP32 狀態
  stop                              立即停止並回到 IDLE
  estop                             鎖定急停
  clear                             清除急停／timeout（輸出仍為 0）
  help                              顯示說明
  quit                              STOP 後離開

肘 PWM 為正：二頭肌 +PWM、三頭肌以 0.7 倍 -PWM 放線。
肘 PWM 為負：三頭肌 +PWM、二頭肌以 0.8 倍 -PWM 放線。
程式禁止單獨驅動二頭肌或三頭肌。
每次輸出最長 {maximum:g} 秒。

例：
  elbow 200 0.5       -> PWM,200,-140,0
  elbow -200 0.5      -> PWM,-160,200,0
  shoulder 200 0.5    -> PWM,0,0,200
  all 200 150 1.0     -> PWM,200,-140,150
""".format(maximum=max_duration)
    )


def show_status(bridge):
    bridge.ping()
    time.sleep(0.2)
    fb = bridge.get_feedback()
    print(
        f"ESP32 state={bridge.safety_state}｜last={bridge.last_line}\n"
        f"feedback PWM=({fb.pwm1}, {fb.pwm2}, {fb.pwm3})"
    )


def run_timed_pwm(bridge, values, duration, stop_event=None, progress_callback=None):
    if not has_valid_antagonist_pair(values):
        raise ValueError("安全規則拒絕輸出：二頭肌與三頭肌必須反向或同時為 0。")
    if not any(values):
        bridge.stop()
        print("三路皆為 0，已送出 STOP。")
        return

    bridge.stop()
    time.sleep(0.05)
    bridge.arm()
    print(f"開始輸出 PWM={values}，{duration:.2f} 秒後自動 STOP。Ctrl+C 可立即停止。")
    started = time.monotonic()
    next_report = started
    try:
        while time.monotonic() - started < duration:
            if stop_event is not None and stop_event.is_set():
                break
            bridge.set_pwm(*values)
            now = time.monotonic()
            if now >= next_report:
                fb = bridge.get_feedback()
                remaining = max(0.0, duration - (now - started))
                print(
                    f"  剩餘 {remaining:4.1f}s｜ESP32 applied="
                    f"({fb.pwm1}, {fb.pwm2}, {fb.pwm3})",
                    end="\r",
                    flush=True,
                )
                next_report = now + 0.25
            if progress_callback is not None:
                progress_callback(now - started, bridge.get_feedback())
            time.sleep(0.05)
    finally:
        bridge.stop()
        time.sleep(0.1)
        print("\n已送出 STOP，輸出歸零。")


def interactive_console(bridge, max_duration):
    print_help(max_duration)
    while True:
        try:
            line = input("motor> ").strip()
        except EOFError:
            line = "quit"
        if not line:
            continue

        try:
            parts = shlex.split(line)
            command = parts[0].lower()
            if command in ("quit", "exit", "q"):
                return
            if command in ("help", "h", "?"):
                print_help(max_duration)
            elif command == "status":
                show_status(bridge)
            elif command == "stop":
                bridge.stop()
                print("已送出 STOP。")
            elif command == "estop":
                bridge.emergency_stop()
                time.sleep(0.1)
                print(f"已送出 ESTOP｜{bridge.last_line}")
            elif command == "clear":
                bridge.clear_fault()
                time.sleep(0.1)
                print(f"已送出 CLEAR，輸出仍為 0｜{bridge.last_line}")
            elif command in ("elbow", "pair"):
                if len(parts) != 3:
                    raise ValueError("格式：elbow <PWM> <秒數>")
                values = paired_pwm(parts[1], 0)
                duration = parse_duration(parts[2], max_duration)
                run_timed_pwm(bridge, values, duration)
            elif command == "shoulder":
                if len(parts) != 3:
                    raise ValueError("格式：shoulder <PWM> <秒數>")
                values = paired_pwm(0, parts[1])
                duration = parse_duration(parts[2], max_duration)
                run_timed_pwm(bridge, values, duration)
            elif command == "all":
                if len(parts) != 4:
                    raise ValueError("格式：all <肘PWM> <肩PWM> <秒數>")
                values = paired_pwm(parts[1], parts[2])
                duration = parse_duration(parts[3], max_duration)
                run_timed_pwm(bridge, values, duration)
            else:
                print("未知指令；輸入 help 查看說明。")
        except KeyboardInterrupt:
            bridge.stop()
            print("\n已中斷並送出 STOP。")
        except Exception as exc:
            try:
                bridge.stop()
            except Exception:
                pass
            print(f"指令失敗，已送 STOP：{exc}")


def create_manual_gui(bridge, max_duration):
    """Create the standalone browser GUI without starting the web server."""
    from flask import Flask, jsonify, request

    app = Flask(__name__)
    state_lock = threading.Lock()
    stop_event = threading.Event()
    state = {
        "running": False,
        "message": "待機中；ESP32 輸出為 STOP。",
        "command": (0, 0, 0),
        "remaining": 0.0,
    }

    page = r"""<!doctype html>
<html lang="zh-Hant"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>三馬達手動測試</title>
<style>
body{margin:0;background:#f1f5f9;color:#172033;font-family:Arial,"Noto Sans TC",sans-serif}
.wrap{max-width:820px;margin:auto;padding:18px}.card{background:#fff;border-radius:14px;padding:18px;margin-bottom:14px;box-shadow:0 3px 14px #0f172a18}
h1{font-size:24px;margin:0 0 8px}h2{font-size:18px;margin:0 0 12px}.warning{border-left:6px solid #dc2626;background:#fef2f2}
.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:12px}label{font-size:13px;color:#475569;display:block;margin-bottom:5px}
input{width:100%;box-sizing:border-box;padding:10px;border:1px solid #cbd5e1;border-radius:8px;font-size:16px}
button{width:100%;border:0;border-radius:9px;padding:11px;margin-top:9px;background:#2563eb;color:#fff;font-size:15px;cursor:pointer}
button:active{transform:scale(.98)}button:disabled{background:#94a3b8}.danger{background:#dc2626}.stop{background:#f59e0b;color:#111827}.clear{background:#64748b}
.status{white-space:pre-wrap;background:#0f172a;color:#e2e8f0;border-radius:10px;padding:13px;line-height:1.55;font-family:Consolas,monospace}
.command{font-size:22px;font-weight:bold;color:#1d4ed8;margin-top:10px}.small{font-size:13px;color:#64748b;line-height:1.5}
@media(max-width:650px){.grid{grid-template-columns:1fr}}
</style></head><body><div class="wrap">
<div class="card warning"><h1>ESP32 三馬達手動測試</h1>
<div>僅限空載／固定測試架，禁止連接人體。輸出前確認繩索方向並把急停放在手邊。</div></div>

<div class="card"><h2>肘關節拮抗參數</h2><div class="grid">
 <div><label>二頭肌收縮 PWM（0–255）</label><input id="bicepsPwm" type="number" min="0" max="255" value="200"></div>
 <div><label>二頭收縮時：三頭放線比例</label><input id="tricepsRelease" type="number" min="0" max="1" step="0.05" value="0.70"></div>
 <div><label>三頭肌收縮 PWM（0–255）</label><input id="tricepsPwm" type="number" min="0" max="255" value="200"></div>
 <div><label>三頭收縮時：二頭放線比例</label><input id="bicepsRelease" type="number" min="0" max="1" step="0.05" value="0.80"></div>
 <div><label>測試時間（0.05–__MAX__ 秒）</label><input id="duration" type="number" min="0.05" max="__MAX__" step="0.05" value="0.50"></div>
 </div>
 <div class="grid"><button onclick="startTest('biceps')">二頭肌收縮／三頭肌放線</button>
 <button onclick="startTest('triceps')">三頭肌收縮／二頭肌放線</button></div>
 <div id="preview" class="command">預覽：PWM,200,-140,0</div>
</div>

<div class="card"><h2>三角肌獨立測試</h2><div class="grid">
 <div><label>三角肌 PWM（0–255）</label><input id="shoulderPwm" type="number" min="0" max="255" value="200"></div>
 <div><button onclick="startTest('shoulder_positive')">三角肌正向＋</button><button onclick="startTest('shoulder_negative')">三角肌反向－</button></div>
 </div></div>

<div class="card"><div class="grid"><button class="stop" onclick="control('stop')">STOP</button>
<button class="danger" onclick="control('estop')">EMERGENCY STOP</button></div>
<button class="clear" onclick="control('clear')">CLEAR 急停／Timeout（不啟動）</button></div>
<div class="card"><h2>即時狀態</h2><div id="status" class="status">連線中...</div>
<p class="small">每 50 ms 重送一次命令；測試時間到、按 STOP、關閉程式或通訊中斷都會停止輸出。</p></div>
</div><script>
const $=id=>document.getElementById(id);const n=id=>Number($(id).value);
function params(){return {biceps_pwm:n('bicepsPwm'),triceps_pwm:n('tricepsPwm'),triceps_release_ratio:n('tricepsRelease'),biceps_release_ratio:n('bicepsRelease'),shoulder_pwm:n('shoulderPwm'),duration:n('duration')}}
function clamp(v,a,b){return Math.max(a,Math.min(b,v))}
function updatePreview(){const p=params(),b=Math.round(clamp(p.biceps_pwm,0,255)),r=clamp(p.triceps_release_ratio,0,1);$('preview').textContent='預覽：PWM,'+b+','+(-Math.round(b*r))+',0'}
document.querySelectorAll('input').forEach(x=>x.addEventListener('input',updatePreview));
async function post(url,data={}){try{const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});return await r.json()}catch(e){return {ok:false,message:'連線失敗：'+e.message}}}
async function startTest(action){const p=params();let text=action==='biceps'?'二頭肌收縮／三頭肌放線':action==='triceps'?'三頭肌收縮／二頭肌放線':'三角肌測試';if(!confirm(text+'\n\n確認機構已固定、沒有連接人體、急停可立即操作？'))return;const d=await post('/api/run',{action,...p});$('status').textContent=d.message||'沒有回覆'}
async function control(action){const d=await post('/api/'+action);$('status').textContent=d.message||'沒有回覆'}
async function poll(){try{const r=await fetch('/api/status'),d=await r.json();$('status').textContent=(d.message||'')+'\ncommand='+d.command+'\nremaining='+Number(d.remaining||0).toFixed(2)+'s\nESP32 state='+d.esp32_state+'\napplied='+d.applied+'\nlast='+d.last_line;document.querySelectorAll('button').forEach(b=>{if(!b.classList.contains('stop')&&!b.classList.contains('danger')&&!b.classList.contains('clear'))b.disabled=d.running})}catch(e){}setTimeout(poll,250)}
updatePreview();poll();
</script></body></html>""".replace("__MAX__", f"{max_duration:g}")

    def bounded_ratio(value, name):
        ratio = float(value)
        if not 0.0 <= ratio <= 1.0:
            raise ValueError(f"{name} 必須介於 0 到 1。")
        return ratio

    def worker(values, duration):
        def progress(elapsed, _feedback):
            with state_lock:
                state["remaining"] = max(0.0, duration - elapsed)

        try:
            run_timed_pwm(
                bridge,
                values,
                duration,
                stop_event=stop_event,
                progress_callback=progress,
            )
            message = "測試完成，已自動 STOP。" if not stop_event.is_set() else "測試已由使用者停止。"
        except Exception as exc:
            message = f"測試失敗並已 STOP：{exc}"
        finally:
            try:
                bridge.stop()
            except Exception:
                pass
            with state_lock:
                state.update(running=False, message=message, command=(0, 0, 0), remaining=0.0)

    @app.get("/")
    def home():
        return page

    @app.post("/api/run")
    def api_run():
        data = request.get_json(silent=True) or {}
        try:
            action = str(data.get("action", ""))
            duration = parse_duration(data.get("duration", 0.5), max_duration)
            triceps_ratio = bounded_ratio(data.get("triceps_release_ratio", 0.7), "三頭放線比例")
            biceps_ratio = bounded_ratio(data.get("biceps_release_ratio", 0.8), "二頭放線比例")
            if action == "biceps":
                pwm = abs(parse_pwm(data.get("biceps_pwm", 200)))
                values = paired_pwm(pwm, 0, triceps_ratio, biceps_ratio)
            elif action == "triceps":
                pwm = abs(parse_pwm(data.get("triceps_pwm", 200)))
                values = paired_pwm(-pwm, 0, triceps_ratio, biceps_ratio)
            elif action in ("shoulder_positive", "shoulder_negative"):
                pwm = abs(parse_pwm(data.get("shoulder_pwm", 200)))
                values = (0, 0, pwm if action == "shoulder_positive" else -pwm)
            else:
                raise ValueError("未知測試動作。")
            if not any(values):
                raise ValueError("PWM 不可全部為 0。")
        except Exception as exc:
            return jsonify(ok=False, message=str(exc)), 400

        with state_lock:
            if state["running"]:
                return jsonify(ok=False, message="已有測試執行中；請先按 STOP。"), 409
            stop_event.clear()
            state.update(running=True, message="測試啟動中...", command=values, remaining=duration)
        threading.Thread(target=worker, args=(values, duration), daemon=True).start()
        return jsonify(ok=True, message=f"開始輸出 PWM,{values[0]},{values[1]},{values[2]}，{duration:.2f} 秒後自動 STOP。")

    @app.post("/api/stop")
    def api_stop():
        stop_event.set()
        bridge.stop()
        with state_lock:
            state["message"] = "已按 STOP，正在停止輸出。"
        return jsonify(ok=True, message="已送出 STOP。")

    @app.post("/api/estop")
    def api_estop():
        stop_event.set()
        bridge.emergency_stop()
        with state_lock:
            state["message"] = "EMERGENCY STOP 已鎖定。"
        return jsonify(ok=True, message="ESTOP 已送出；必須 CLEAR 才能再次測試。")

    @app.post("/api/clear")
    def api_clear():
        stop_event.set()
        bridge.clear_fault()
        bridge.stop()
        with state_lock:
            state["message"] = "故障已清除，輸出維持 STOP。"
        return jsonify(ok=True, message="已送 CLEAR 與 STOP。")

    @app.get("/api/status")
    def api_status():
        feedback = bridge.get_feedback()
        with state_lock:
            snapshot = dict(state)
        snapshot.update(
            command=",".join(str(value) for value in snapshot["command"]),
            esp32_state=bridge.safety_state,
            applied=f"{feedback.pwm1},{feedback.pwm2},{feedback.pwm3}",
            last_line=bridge.last_line,
        )
        return jsonify(snapshot)

    return app


def main():
    parser = argparse.ArgumentParser(description="ESP32 三馬達 USB PWM 手動測試工具")
    parser.add_argument("--port", help="例如 /dev/ttyUSB0；省略時自動尋找")
    parser.add_argument("--gui", action="store_true", help="啟動瀏覽器 GUI（預設 port 5001）")
    parser.add_argument("--host", default="0.0.0.0", help="GUI 監聽位址，預設 0.0.0.0")
    parser.add_argument("--gui-port", type=int, default=5001, help="GUI TCP port，預設 5001")
    parser.add_argument(
        "--max-duration",
        type=float,
        default=5.0,
        help="單次輸出最長秒數，預設 5 秒、最高 30 秒",
    )
    args = parser.parse_args()
    if not 0.1 <= args.max_duration <= 30.0:
        parser.error("--max-duration 必須介於 0.1 到 30 秒")
    if not 1 <= args.gui_port <= 65535:
        parser.error("--gui-port 必須介於 1 到 65535")

    if service_is_active():
        print(
            "imu-control.service 仍在執行，為避免兩個程式同時控制 USB Serial，"
            "請先執行：\n  sudo systemctl stop imu-control.service",
            file=sys.stderr,
        )
        return 2

    print("警告：僅限空載／固定測試架，禁止連接人體；請把急停放在手邊。")
    bridge = None
    try:
        bridge = connect_bridge(args.port)
        if args.gui:
            app = create_manual_gui(bridge, args.max_duration)
            print(f"手動馬達 GUI：http://樹莓派IP:{args.gui_port}")
            app.run(
                host=args.host,
                port=args.gui_port,
                debug=False,
                threaded=True,
                use_reloader=False,
            )
        else:
            interactive_console(bridge, args.max_duration)
        return 0
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在停止。")
        return 130
    except Exception as exc:
        print(f"錯誤：{exc}", file=sys.stderr)
        return 1
    finally:
        if bridge is not None:
            try:
                bridge.stop()
                time.sleep(0.1)
            except Exception:
                pass
            bridge.close()
        print("程式結束；已嘗試送出 STOP。")


if __name__ == "__main__":
    raise SystemExit(main())
