#!/usr/bin/env python3
"""Serve one local Web UI for dual-arm health monitoring and recorder control."""

from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any


ARM_STATE_SCHEMA = "flexiv_dual_arm_state.v1"
RECORDER_STATUS_SCHEMA = "flexiv_recorder_status.v1"
RECORDER_COMMANDS = {"start", "pause", "save", "discard", "reset", "quit"}


class LatestPacket:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._packet: dict[str, Any] | None = None
        self._received_at = 0.0

    def update(self, packet: dict[str, Any]) -> None:
        with self._lock:
            self._packet = packet
            self._received_at = time.monotonic()

    def snapshot(self, *, online_timeout_sec: float) -> dict[str, Any]:
        with self._lock:
            packet = None if self._packet is None else dict(self._packet)
            received_at = self._received_at
        age = None if received_at <= 0.0 else max(0.0, time.monotonic() - received_at)
        return {
            "online": age is not None and age <= float(online_timeout_sec),
            "age_sec": age,
            "data": packet,
        }


class UdpJsonReceiver(threading.Thread):
    def __init__(self, host: str, port: int, schema: str, target: LatestPacket) -> None:
        super().__init__(daemon=True)
        self.host = str(host)
        self.port = int(port)
        self.schema = str(schema)
        self.target = target
        self.stop_event = threading.Event()
        self.socket: socket.socket | None = None

    def run(self) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.host, self.port))
        self.socket.settimeout(0.5)
        while not self.stop_event.is_set():
            try:
                payload, _address = self.socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                packet = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if isinstance(packet, dict) and packet.get("schema") == self.schema:
                self.target.update(packet)

    def close(self) -> None:
        self.stop_event.set()
        if self.socket is not None:
            self.socket.close()


class DashboardState:
    def __init__(self, command_host: str, command_port: int, *, timeout_sec: float) -> None:
        self.arm = LatestPacket()
        self.recorder = LatestPacket()
        self.command_address = (str(command_host), int(command_port))
        self.timeout_sec = float(timeout_sec)

    def status(self) -> dict[str, Any]:
        return {
            "server_time": time.time(),
            "arm": self.arm.snapshot(online_timeout_sec=self.timeout_sec),
            "recorder": self.recorder.snapshot(online_timeout_sec=self.timeout_sec),
        }

    def send_recorder_command(self, command: str) -> None:
        command = str(command).strip().lower()
        if command not in RECORDER_COMMANDS:
            raise ValueError(f"unsupported recorder command: {command}")
        payload = json.dumps({"command": command}, separators=(",", ":")).encode("utf-8")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(payload, self.command_address)


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "FlexivDashboard/1.0"

    @property
    def dashboard(self) -> DashboardState:
        return self.server.dashboard  # type: ignore[attr-defined]

    def _json_response(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        path = self.path.split("?", 1)[0]
        if path == "/":
            body = DASHBOARD_HTML.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if path in ("/api/status", "/healthz"):
            self._json_response(HTTPStatus.OK, self.dashboard.status())
            return
        self._json_response(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        path = self.path.split("?", 1)[0]
        if path == "/api/reset":
            self.dashboard.send_recorder_command("reset")
            self._json_response(HTTPStatus.ACCEPTED, {"ok": True, "command": "reset"})
            return
        if path != "/api/recorder":
            self._json_response(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 4096:
                raise ValueError("invalid request length")
            request = json.loads(self.rfile.read(length).decode("utf-8"))
            command = request.get("command") if isinstance(request, dict) else None
            self.dashboard.send_recorder_command(str(command))
        except (ValueError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            self._json_response(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return
        self._json_response(HTTPStatus.ACCEPTED, {"ok": True, "command": command})

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[web-dashboard] {self.address_string()} {fmt % args}", flush=True)


class DashboardHttpServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], dashboard: DashboardState) -> None:
        super().__init__(address, DashboardHandler)
        self.dashboard = dashboard


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--arm-state-host", default="127.0.0.1")
    parser.add_argument("--arm-state-port", type=int, default=57684)
    parser.add_argument("--recorder-status-host", default="127.0.0.1")
    parser.add_argument("--recorder-status-port", type=int, default=57688)
    parser.add_argument("--recorder-command-host", default="127.0.0.1")
    parser.add_argument("--recorder-command-port", type=int, default=57687)
    parser.add_argument("--online-timeout-sec", type=float, default=2.0)
    args = parser.parse_args(argv)
    for name in ("port", "arm_state_port", "recorder_status_port", "recorder_command_port"):
        port = int(getattr(args, name))
        if not 0 < port <= 65535:
            parser.error(f"--{name.replace('_', '-')} must be between 1 and 65535")
    if args.online_timeout_sec <= 0.0:
        parser.error("--online-timeout-sec must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    dashboard = DashboardState(
        args.recorder_command_host,
        args.recorder_command_port,
        timeout_sec=args.online_timeout_sec,
    )
    receivers = [
        UdpJsonReceiver(args.arm_state_host, args.arm_state_port, ARM_STATE_SCHEMA, dashboard.arm),
        UdpJsonReceiver(
            args.recorder_status_host,
            args.recorder_status_port,
            RECORDER_STATUS_SCHEMA,
            dashboard.recorder,
        ),
    ]
    for receiver in receivers:
        receiver.start()
    server = DashboardHttpServer((args.host, args.port), dashboard)
    print(
        f"[web-dashboard] listening on http://{args.host}:{args.port}; "
        f"arm_udp={args.arm_state_host}:{args.arm_state_port} "
        f"recorder_udp={args.recorder_status_host}:{args.recorder_status_port}",
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        for receiver in receivers:
            receiver.close()
        for receiver in receivers:
            receiver.join(timeout=1.0)
    return 0


DASHBOARD_HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Flexiv 双臂采集台</title>
<style>
:root{color-scheme:dark;--bg:#0a0d12;--panel:#121821;--line:#263142;--text:#e7edf5;--muted:#91a0b3;--green:#42d392;--red:#ff6577;--amber:#f7c65e;--blue:#6aa8ff}
*{box-sizing:border-box}html{scrollbar-gutter:stable}body{margin:0;background:radial-gradient(circle at top,#162132 0,var(--bg) 42%);font:14px/1.45 ui-monospace,SFMono-Regular,Menlo,monospace;color:var(--text);overflow-anchor:none;font-variant-numeric:tabular-nums}
main{max-width:1500px;margin:auto;padding:20px}.top{display:flex;align-items:center;justify-content:space-between;gap:16px}.title{font:700 24px system-ui;margin:0}.sub{color:var(--muted)}
.badges,.buttons{display:flex;gap:8px;flex-wrap:wrap}.badge{padding:6px 10px;border:1px solid var(--line);border-radius:999px;background:#0c121a;color:var(--muted)}.ok{color:var(--green);border-color:#245c48}.bad{color:var(--red);border-color:#6a2e39}.warn{color:var(--amber);border-color:#665329}
.grid{display:grid;grid-template-columns:repeat(12,1fr);gap:14px;margin-top:14px}.panel{background:rgba(18,24,33,.94);border:1px solid var(--line);border-radius:12px;padding:14px;box-shadow:0 12px 30px #0005}.record{grid-column:span 5}.health{grid-column:span 7}.arm{grid-column:span 6}.chart{grid-column:span 12}
h2{font:650 16px system-ui;margin:0 0 12px}.metric{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}.cell{background:#0b1119;border:1px solid #1e2938;border-radius:8px;padding:9px}.label{color:var(--muted);font-size:11px}.value{font:650 17px system-ui;margin-top:3px;min-height:25px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;font-variant-numeric:tabular-nums}
button{border:1px solid #34445a;background:#192333;color:var(--text);padding:10px 14px;border-radius:8px;font:650 13px system-ui;cursor:pointer}button:hover{border-color:var(--blue)}button.primary{background:#165f43;border-color:#2c9e70}button.danger{background:#54212a;border-color:#9e3d4e}button:disabled{opacity:.35;cursor:not-allowed}
.system-reset{display:flex;align-items:center;gap:12px;margin-top:12px;padding:10px;border:1px solid #665329;border-radius:8px;background:#241d0d}.system-reset button{background:#6b4d10;border-color:#c79529;min-width:190px}.system-reset .hint{color:var(--amber);font-size:12px}
table{width:100%;border-collapse:collapse;font-size:12px}th,td{text-align:right;padding:5px 6px;border-bottom:1px solid #202a38}th:first-child,td:first-child{text-align:left}th{color:var(--muted);font-weight:500}.hot{color:var(--red);font-weight:700}.safe{color:var(--green)}
canvas{width:100%;height:180px;background:#090e15;border-radius:8px}.event{color:var(--muted);margin-top:9px;min-height:20px}.error{color:var(--red);white-space:pre-wrap}.quest-row{display:grid;grid-template-columns:repeat(2,1fr);gap:8px;margin-top:10px}
@media(max-width:900px){.record,.health,.arm,.chart{grid-column:span 12}.metric{grid-template-columns:repeat(2,1fr)}.top{align-items:flex-start;flex-direction:column}}
</style></head>
<body><main>
<div class="top"><div><h1 class="title">Flexiv 双臂采集台</h1><div class="sub">NRT 遥操 · Quest · 双臂状态 · 数据录制</div></div><div class="badges"><span id="armOnline" class="badge">ARM OFFLINE</span><span id="recOnline" class="badge">RECORDER OFFLINE</span><span id="frameLock" class="badge">QUEST WAIT</span></div></div>
<div class="grid">
 <section class="panel record"><h2>录制控制</h2><div class="metric">
  <div class="cell"><div class="label">状态</div><div id="recState" class="value">—</div></div><div class="cell"><div class="label">当前帧</div><div id="frames" class="value">0</div></div>
  <div class="cell"><div class="label">当前时长</div><div id="duration" class="value">00:00:00</div></div><div class="cell"><div class="label">已保存</div><div id="saved" class="value">0</div></div>
 </div><div class="buttons" style="margin-top:12px"><button class="primary" onclick="command('start')">开始 / 继续</button><button onclick="command('pause')">暂停</button><button class="primary" onclick="command('save')">保存本条</button><button class="danger" onclick="command('discard',true)">丢弃本条</button><button class="danger" onclick="command('quit',true)">停止录制器</button></div><div id="recEvent" class="event"></div><div id="recError" class="error"></div></section>
 <section class="panel health"><h2>采集门控</h2><div class="metric">
  <div class="cell"><div class="label">左臂</div><div id="leftReady" class="value">—</div></div><div class="cell"><div class="label">右臂</div><div id="rightReady" class="value">—</div></div>
  <div class="cell"><div class="label">Quest tracking</div><div id="tracking" class="value">—</div></div><div class="cell"><div class="label">最大力矩比例</div><div id="peakRatio" class="value">—</div></div>
 </div><div class="quest-row"><div class="cell"><div class="label">左手 squeeze / trigger</div><div id="leftQuest" class="value">—</div></div><div class="cell"><div class="label">右手 squeeze / trigger</div><div id="rightQuest" class="value">—</div></div></div><div class="quest-row"><div class="cell"><div class="label">双手距离 ↔ 双臂间距</div><div id="questSpacing" class="value">WAIT</div></div><div class="cell"><div class="label">双手方向 ↔ TCP 朝向</div><div id="questDirection" class="value">WAIT</div></div></div><form class="system-reset" method="post" action="/api/reset" target="resetSink"><button type="submit">RESET 双臂 + 环境</button><span class="hint">单击立即执行：停止控制、回 initial_q、复位场景资产；完成后需双手 squeeze 重新标定。</span></form><iframe name="resetSink" title="reset result" hidden></iframe><div id="healthText" class="event"></div></section>
 <section class="panel arm"><h2>左臂 q / dq / τ</h2><table><thead><tr><th>Joint</th><th>q rad</th><th>dq rad/s</th><th>τ Nm</th><th>τext</th><th>risk</th></tr></thead><tbody id="leftTable"></tbody></table><div id="leftTcp" class="event"></div></section>
 <section class="panel arm"><h2>右臂 q / dq / τ</h2><table><thead><tr><th>Joint</th><th>q rad</th><th>dq rad/s</th><th>τ Nm</th><th>τext</th><th>risk</th></tr></thead><tbody id="rightTable"></tbody></table><div id="rightTcp" class="event"></div></section>
 <section class="panel chart"><h2>实时最大关节力矩风险（红线 0.72）</h2><canvas id="chart" width="1400" height="180"></canvas></section>
</div></main>
<script>
const $=id=>document.getElementById(id),hist={left:[],right:[]};let lastStamp=null,refreshing=false;
const num=(v,n=3)=>Number.isFinite(Number(v))?Number(v).toFixed(n):'—';
const timeFmt=s=>{s=Math.max(0,Math.round(Number(s)||0));return [Math.floor(s/3600),Math.floor(s%3600/60),s%60].map(v=>String(v).padStart(2,'0')).join(':')};
function badge(el,on,good='OK',bad='OFFLINE'){el.textContent=on?good:bad;el.className='badge '+(on?'ok':'bad')}
function quest(a){return a&&a.quest||{}}
function torque(a){return a&&a.torque||{}}
function peak(a){const r=torque(a).ratio||[];return r.length?Math.max(...r.map(Number)):0}
function armTable(side,a){const q=a.q||[],dq=a.dq||[],t=torque(a),tau=t.tau||[],ext=t.tau_ext||[],ratio=t.ratio||[],body=$(side+'Table');if(!body.rows.length)body.innerHTML=Array.from({length:7},(_,i)=>`<tr><td>J${i+1}</td><td></td><td></td><td></td><td></td><td></td></tr>`).join('');Array.from(body.rows).forEach((row,i)=>{const c=row.cells;c[1].textContent=num(q[i],4);c[2].textContent=num(dq[i],4);c[3].textContent=num(tau[i],3);c[4].textContent=num(ext[i],3);c[5].textContent=num(ratio[i],3);c[5].className=Number(ratio[i])>=.72?'hot':'safe'});const p=a.tcp_pose_base||[];$(side+'Tcp').textContent=`TCP base xyz = [${p.slice(0,3).map(v=>num(v,4)).join(', ')}] · phase ${a.phase||'—'} · torque guard ${t.frozen?'FROZEN':'normal'}`}
function draw(){const c=$('chart'),x=c.getContext('2d'),w=c.width,h=c.height;x.clearRect(0,0,w,h);x.strokeStyle='#263142';x.lineWidth=1;for(let r=0;r<=1;r+=.25){const y=h-r*h;x.beginPath();x.moveTo(0,y);x.lineTo(w,y);x.stroke()}x.strokeStyle='#ff6577';x.setLineDash([7,5]);let y=h-.72*h;x.beginPath();x.moveTo(0,y);x.lineTo(w,y);x.stroke();x.setLineDash([]);[['left','#42d392'],['right','#6aa8ff']].forEach(([s,col])=>{const a=hist[s];x.strokeStyle=col;x.lineWidth=2;x.beginPath();a.forEach((v,i)=>{const px=i*w/299,py=h-Math.min(1.05,v)*h;i?x.lineTo(px,py):x.moveTo(px,py)});x.stroke()})}
async function command(name,danger=false){if(danger&&!confirm(`确认执行 ${name}？`))return;try{const r=await fetch('/api/recorder',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command:name})});const b=await r.json();if(!r.ok)throw Error(b.error||r.statusText);$('recEvent').textContent=`已发送命令：${name}`}catch(e){$('recError').textContent=e.message}}
async function refresh(){if(refreshing)return;refreshing=true;try{const s=await fetch('/api/status',{cache:'no-store'}).then(r=>r.json()),ap=s.arm.data||{},rp=s.recorder.data||{},arms=ap.arms||{},l=arms.left||{},r=arms.right||{},lq=quest(l),rq=quest(r),g=lq.calibration_geometry||rq.calibration_geometry||{};badge($('armOnline'),s.arm.online,'ARM ONLINE');badge($('recOnline'),s.recorder.online,'RECORDER ONLINE');const locked=!!lq.calibration_confirmed&&!!rq.calibration_confirmed;badge($('frameLock'),locked,'QUEST TRACKING','QUEST WAIT');$('leftReady').textContent=l.ready?'READY':(l.phase||'WAIT');$('leftReady').className='value '+(l.ready?'safe':'hot');$('rightReady').textContent=r.ready?'READY':(r.phase||'WAIT');$('rightReady').className='value '+(r.ready?'safe':'hot');const tracked=!!lq.motion_data_ready&&!!rq.motion_data_ready;$('tracking').textContent=tracked?'PASS':'WAIT';$('tracking').className='value '+(tracked?'safe':'hot');const pk=Math.max(peak(l),peak(r));$('peakRatio').textContent=num(pk,3);$('peakRatio').className='value '+(pk>=.72?'hot':'safe');$('leftQuest').textContent=`${num(lq.enable_value,2)} / ${num(lq.gripper_value,2)}`;$('rightQuest').textContent=`${num(rq.enable_value,2)} / ${num(rq.gripper_value,2)}`;const spacingOk=!!g.available&&!!g.spacing_ok,directionOk=!!g.available&&!!g.direction_ok;$('questSpacing').textContent=g.available?`${spacingOk?'PASS':'FAIL'} ${num(g.separation_m,3)}m / ${num(g.separation_target_m,2)}±${num(g.separation_tolerance_m,2)}m`:'WAIT';$('questSpacing').className='value '+(spacingOk?'safe':'hot');const directionError=Math.max(Number(g.left_direction_error_deg)||0,Number(g.right_direction_error_deg)||0,Number(g.mutual_direction_error_deg)||0);$('questDirection').textContent=g.available?`${directionOk?'PASS':'FAIL'} max ${num(directionError,1)}° / ${num(g.direction_tolerance_deg,0)}°`:'WAIT';$('questDirection').className='value '+(directionOk?'safe':'hot');const armError=l.error||r.error||'';$('healthText').textContent=`cycle ${ap.servo_cycle??ap.cycle??'—'} · state age ${s.arm.age_sec==null?'—':num(s.arm.age_sec,2)+'s'} · phases ${l.phase||'—'} / ${r.phase||'—'} · ${armError||(locked?'已锁定：保持 squeeze 直接跟随':(spacingOk&&directionOk?'保持对齐并双手 squeeze 0.25s，随后直接跟随':'先对齐双手距离与方向'))}`;armTable('left',l);armTable('right',r);$('recState').textContent=rp.state||'—';$('frames').textContent=rp.frames_current??0;$('duration').textContent=timeFmt(rp.duration_current_sec);$('saved').textContent=`${rp.saved_episodes??0} / ${rp.episodes_target??'—'}`;$('recEvent').textContent=rp.event?`${rp.task_name||''} · ${rp.event}`:'等待 recorder 状态';$('recError').textContent=rp.error||'';const stamp=ap.monotonic_time??ap.servo_cycle??ap.cycle;if(stamp!=null&&stamp!==lastStamp){lastStamp=stamp;hist.left.push(peak(l));hist.right.push(peak(r));if(hist.left.length>300){hist.left.shift();hist.right.shift()}draw()}}catch(e){badge($('armOnline'),false,'','WEB ERROR');$('healthText').textContent=e.message}finally{refreshing=false}}
setInterval(refresh,250);refresh();
</script></body></html>"""


if __name__ == "__main__":
    raise SystemExit(main())
