#!/usr/bin/env python3
from flask import Flask, render_template_string, jsonify
import psutil
import subprocess
from datetime import datetime

app = Flask(__name__)

HTML = '''<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SYSTEM MONITOR</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@300;400;500&family=Space+Mono:wght@400;700&display=swap" rel="stylesheet">
<style>
:root{--black:#000;--surface:#111;--border:#222;--text-disabled:#666;--text-secondary:#999;--text-primary:#E8E8E8;--text-display:#FFF;--accent:#D71921;--success:#4A9E5C}
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Space Grotesk',system-ui,sans-serif;background:var(--black);color:var(--text-primary);min-height:100vh;padding:32px}
.container{max-width:960px;margin:0 auto}
header{margin-bottom:48px;border-bottom:1px solid var(--border);padding-bottom:24px}
h1{font-family:'Space Mono',monospace;font-size:14px;font-weight:400;letter-spacing:0.08em;color:var(--text-secondary);text-transform:uppercase}
.time{font-family:'Space Mono',monospace;font-size:11px;color:var(--text-disabled);letter-spacing:0.04em;margin-top:32px}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:1px;background:var(--border)}
.card{background:var(--surface);padding:24px;min-height:160px}
.card-header{font-family:'Space Mono',monospace;font-size:11px;font-weight:700;letter-spacing:0.08em;color:var(--text-disabled);text-transform:uppercase;margin-bottom:24px}
.stat{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:8px}
.label{font-family:'Space Mono',monospace;font-size:11px;letter-spacing:0.06em;color:var(--text-secondary)}
.val{font-family:'Space Mono',monospace;font-size:24px;font-weight:700;color:var(--text-display);letter-spacing:-0.02em}
.unit{font-size:12px;color:var(--text-secondary);margin-left:4px}
.bar{height:4px;background:var(--border);margin:12px 0;position:relative;overflow:hidden}
.bar-fill{height:100%;background:var(--success);transition:width .3s ease-out}
.bar-fill.mid{background:#D4A843}
.bar-fill.high{background:var(--accent)}
.hero{font-family:'Space Mono',monospace;font-size:48px;font-weight:700;letter-spacing:-0.03em;color:var(--text-display);text-align:center;line-height:1}
.hero-label{font-family:'Space Mono',monospace;font-size:10px;letter-spacing:0.1em;color:var(--text-disabled);text-transform:uppercase;text-align:center;margin-top:8px}
.status{display:flex;align-items:center;gap:8px;padding:12px 0}
.status-dot{width:8px;height:8px;border-radius:50%;background:var(--text-disabled)}
.status-dot.on{background:var(--success)}
.status-text{font-family:'Space Mono',monospace;font-size:12px;letter-spacing:0.02em}
.toggle-btn{font-family:'Space Mono',monospace;font-size:10px;letter-spacing:0.06em;text-transform:uppercase;background:var(--border);color:var(--text-secondary);border:none;padding:6px 12px;cursor:pointer;transition:.2s;border-radius:4px}
.toggle-btn:hover{background:var(--text-disabled);color:var(--text-primary)}
@media(max-width:768px){.grid{grid-template-columns:1fr}}
</style>
</head>
<body>
<div class="container">
<header>
<h1>RASPBERRY PI MONITOR</h1>
</header>
<div class="grid">
<div class="card"><div class="card-header">CPU</div><div class="stat"><span class="label">USAGE</span><span class="val" id="cpu">--<span class="unit">%</span></span></div><div class="bar"><div class="bar-fill" id="cbar"></div></div><div class="hero" id="temp">--<span class="unit">°C</span></div><div class="hero-label">TEMP</div></div>
<div class="card"><div class="card-header">MEMORY</div><div class="stat"><span class="label">USED</span><span class="val" id="mem">--<span class="unit">GB</span></span></div><div class="bar"><div class="bar-fill" id="mbar"></div></div><div class="stat"><span class="label">TOTAL</span><span class="label" id="memTotal">--GB</span></div></div>
<div class="card" style="min-height:180px"><div class="card-header">STORAGE</div><div id="diskList" style="display:flex;flex-direction:column;gap:12px"></div></div>
<div class="card"><div class="card-header">UPTIME</div><div class="hero" id="up">--</div><div class="hero-label">RUNNING</div></div>
<a href="http://192.168.1.253:2017/" target="_blank" style="text-decoration:none;color:inherit;display:contents"><div class="card" style="cursor:pointer"><div class="card-header">PROXY</div><div class="status"><div class="status-dot" id="vpnDot"></div><span class="status-text" id="vpnStatus">OFFLINE</span></div></div></a>
<div class="card"><div class="card-header">NETWORK</div><div class="stat"><span class="label">UP</span><span class="val" id="sent" style="font-size:16px">--</span></div><div class="stat"><span class="label">DOWN</span><span class="val" id="recv" style="font-size:16px">--</span></div></div>
<div class="card"><div class="card-header">IM BRIDGE</div><div class="status"><div class="status-dot" id="bridgeDot"></div><span class="status-text" id="bridgeStatus">OFFLINE</span></div><button class="toggle-btn" id="bridgeBtn" onclick="toggleService('bridge')">TOGGLE</button></div>
<div class="card"><div class="card-header">OPENCLAW</div><div class="status"><div class="status-dot" id="openclawDot"></div><span class="status-text" id="openclawStatus">OFFLINE</span></div><button class="toggle-btn" id="openclawBtn" onclick="toggleService('openclaw')">TOGGLE</button></div>
<div class="card"><div class="card-header">CAR DASHBOARD</div><div class="status"><div class="status-dot" id="carDashboardDot"></div><span class="status-text" id="carDashboardStatus">OFFLINE</span></div><button class="toggle-btn" id="carDashboardBtn" onclick="toggleService('car_dashboard')">TOGGLE</button></div>
<div class="card"><div class="card-header">CAR VIDEO</div><div class="status"><div class="status-dot" id="carVideoDot"></div><span class="status-text" id="carVideoStatus">OFFLINE</span></div><button class="toggle-btn" id="carVideoBtn" onclick="toggleService('car_video')">TOGGLE</button></div>
<div class="card"><div class="card-header">GOOFISH BOT</div><div class="status"><div class="status-dot" id="goofishDot"></div><span class="status-text" id="goofishStatus">OFFLINE</span></div><button class="toggle-btn" id="goofishBtn" onclick="toggleService('goofish')">TOGGLE</button></div>
</div>
<div class="time">UPDATE: <span id="t">--</span></div>
</div>
<script>
function f(b){if(!b)return"0 B";const k=1024,s=["B","KB","MB","GB"];let i=Math.floor(Math.log(b)/Math.log(k));return(b/Math.pow(k,i)).toFixed(1)+" "+s[i];}
function bc(p){return"bar-fill"+(p>80?" high":p>50?" mid":"")}
function u(){fetch("/s").then(r=>r.json()).then(d=>{
document.getElementById("cpu").innerHTML=d.c.u.toFixed(1)+'<span class="unit">%</span>';
var cb=document.getElementById("cbar");cb.style.width=d.c.u+"%";cb.className=bc(d.c.u);
document.getElementById("temp").innerHTML=(d.c.t?d.c.t.toFixed(1):"--")+'<span class="unit">°C</span>';
document.getElementById("mem").innerHTML=d.mu+'<span class="unit">GB</span>';
document.getElementById("memTotal").innerText=d.mt+"GB";
var mb=document.getElementById("mbar");mb.style.width=d.mp+"%";mb.className=bc(d.mp);
var dl=document.getElementById("diskList");dl.innerHTML="";d.disks.forEach(function(disk){
var pct=disk.percent,cls=bc(pct);
dl.innerHTML+='<div><div style="display:flex;justify-content:space-between"><span class="label">'+disk.name+'</span><span class="label">'+disk.used+'/'+disk.total+'GB</span></div><div class="bar"><div class="bar-fill '+cls+'" style="width:'+pct+'%"></div></div></div>';
});
document.getElementById("up").innerText=d.u;
document.getElementById("sent").innerText=f(d.ns);
document.getElementById("recv").innerText=f(d.nr);
var v=document.getElementById("vpnDot"),s=document.getElementById("vpnStatus");
v.className="status-dot"+(d.vpn?" on":"");s.innerText=d.vpn?"ONLINE":"OFFLINE";s.style.color=d.vpn?"var(--success)":"var(--text-disabled)";
var bd=document.getElementById("bridgeDot"),bs=document.getElementById("bridgeStatus");
bd.className="status-dot"+(d.bridge?" on":"");bs.innerText=d.bridge?"ONLINE":"OFFLINE";bs.style.color=d.bridge?"var(--success)":"var(--text-disabled)";
var od=document.getElementById("openclawDot"),os=document.getElementById("openclawStatus");
od.className="status-dot"+(d.openclaw?" on":"");os.innerText=d.openclaw?"ONLINE":"OFFLINE";os.style.color=d.openclaw?"var(--success)":"var(--text-disabled)";
var cdd=document.getElementById("carDashboardDot"),cds=document.getElementById("carDashboardStatus");
cdd.className="status-dot"+(d.car_dashboard?" on":"");cds.innerText=d.car_dashboard?"ONLINE":"OFFLINE";cds.style.color=d.car_dashboard?"var(--success)":"var(--text-disabled)";
var cvd=document.getElementById("carVideoDot"),cvs=document.getElementById("carVideoStatus");
cvd.className="status-dot"+(d.car_video?" on":"");cvs.innerText=d.car_video?"ONLINE":"OFFLINE";cvs.style.color=d.car_video?"var(--success)":"var(--text-disabled)";
var gd=document.getElementById("goofishDot"),gs=document.getElementById("goofishStatus");
gd.className="status-dot"+(d.goofish?" on":"");gs.innerText=d.goofish?"ONLINE":"OFFLINE";gs.style.color=d.goofish?"var(--success)":"var(--text-disabled)";
document.getElementById("t").innerText=d.tm;
});}
u();setInterval(u,3000);
function toggleService(svc){
var btn=document.getElementById(svc+"Btn");btn.disabled=true;btn.innerText="...";
fetch("/ctrl/"+svc,{method:"POST"}).then(r=>r.json()).then(d=>{setTimeout(u,500);btn.disabled=false;btn.innerText="TOGGLE";});
}
</script>
</body>
</html>'''

def get_data():
    t = None
    try:
        for k in psutil.sensors_temperatures():
            if 'cpu' in k.lower(): t = psutil.sensors_temperatures()[k][0].current; break
    except: pass
    m = psutil.virtual_memory()
    disks = []
    try:
        root = psutil.disk_usage('/')
        disks.append({'total': round(root.total / 1e9, 1), 'used': round(root.used / 1e9, 1), 'percent': root.percent, 'mount': '/', 'name': 'Root'})
        for part in psutil.disk_partitions():
            if part.mountpoint.startswith('/media') or part.mountpoint.startswith('/mnt'):
                try:
                    usage = psutil.disk_usage(part.mountpoint)
                    disks.append({'total': round(usage.total / 1e9, 1), 'used': round(usage.used / 1e9, 1), 'percent': usage.percent, 'mount': part.mountpoint, 'name': part.mountpoint.split('/')[-1]})
                except: pass
    except: pass
    n = psutil.net_io_counters()
    b = psutil.boot_time()
    s = datetime.now().timestamp() - b
    d, h = int(s // 86400), int((s % 86400) // 3600)
    m2 = int((s % 3600) // 60)
    up = f'{d}天{h}小时' if d else f'{h}小时{m2}分钟' if h else f'{m2}分钟'
    v2ray_running = True if subprocess.run(['pgrep', '-x', 'v2ray'], capture_output=True).returncode == 0 or subprocess.run(['pgrep', '-f', 'v2raya'], capture_output=True).returncode == 0 else False
    bridge_running = subprocess.run(['systemctl', 'is-active', 'opencode-im-bridge.service'], capture_output=True, text=True).stdout.strip() == 'active'
    openclaw_running = subprocess.run(['systemctl', 'is-active', 'openclaw.service'], capture_output=True, text=True).stdout.strip() == 'active' or subprocess.run(['pgrep', '-f', 'openclaw'], capture_output=True).returncode == 0
    car_dashboard = subprocess.run(['systemctl', 'is-active', 'rpi-car-dashboard.service'], capture_output=True, text=True).stdout.strip() == 'active'
    car_video = subprocess.run(['systemctl', 'is-active', 'rpi-car-video.service'], capture_output=True, text=True).stdout.strip() == 'active'
    goofish_running = subprocess.run(['pgrep', '-f', 'goofishbot/dist/index.js'], capture_output=True).returncode == 0
    return {'c': {'u': psutil.cpu_percent(0.5), 't': t}, 'mt': round(m.total / 1e9, 1), 'mu': round(m.used / 1e9, 1), 'mp': m.percent, 'disks': disks, 'ns': n.bytes_sent, 'nr': n.bytes_recv, 'u': up, 'tm': datetime.now().strftime('%H:%M:%S'), 'vpn': v2ray_running, 'bridge': bridge_running, 'openclaw': openclaw_running, 'car_dashboard': car_dashboard, 'car_video': car_video, 'goofish': goofish_running}

@app.route('/')
def index(): return render_template_string(HTML)
@app.route('/s')
def api(): return jsonify(get_data())
@app.route('/ctrl/<svc>', methods=['POST'])
def ctrl(svc):
    svcs={'bridge':'opencode-im-bridge.service','openclaw':'openclaw.service','car_dashboard':'rpi-car-dashboard.service','car_video':'rpi-car-video.service','goofish':'goofishcbot'}
    name=svcs.get(svc)
    if not name: return jsonify({'error':'unknown'}),400
    if svc == 'goofish':
        running = subprocess.run(['pgrep','-f','goofishbot'],capture_output=True).returncode==0
        cmd='stop' if running else 'start'
        r=subprocess.run(['/home/yoi/.nvm/versions/node/v20.20.0/bin/node','/home/yoi/.npm/_npx/5f7878ce38f1eb13/node_modules/pm2/bin/pm2',cmd,'goofishcbot'],capture_output=True,text=True)
    else:
        active=subprocess.run(['systemctl','is-active',name],capture_output=True,text=True).stdout.strip()=='active'
        cmd='stop' if active else 'start'
        r=subprocess.run(['sudo','systemctl',cmd,name],capture_output=True,text=True)
    if r.returncode!=0: return jsonify({'error':r.stderr.strip()}),500
    return jsonify({'status':cmd+'ed'})

if __name__ == '__main__':
    print("启动: http://192.168.1.253:8080")
    app.run(host='0.0.0.0', port=8080, debug=False)
