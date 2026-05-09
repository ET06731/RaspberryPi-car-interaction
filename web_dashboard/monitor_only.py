#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
轻量系统监控 - 无GPIO依赖
端口: 5001
"""

from flask import Flask, render_template_string, jsonify
import psutil
import os
from datetime import datetime

app = Flask(__name__)

HTML = '''
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>树莓派监控</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; padding: 20px; }
        .container { max-width: 900px; margin: 0 auto; }
        h1 { color: white; text-align: center; margin-bottom: 30px; font-size: 2em; }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 15px; }
        .card { background: white; border-radius: 15px; padding: 20px; box-shadow: 0 10px 30px rgba(0,0,0,0.2); }
        .card h3 { color: #333; margin-bottom: 15px; padding-bottom: 10px; border-bottom: 2px solid #667eea; }
        .stat { display: flex; justify-content: space-between; padding: 8px 0; }
        .label { color: #666; }
        .value { font-weight: bold; color: #333; }
        .bar { height: 20px; background: #eee; border-radius: 10px; overflow: hidden; margin: 10px 0; }
        .fill { height: 100%; transition: width 0.5s; border-radius: 10px; }
        .cpu-fill { background: linear-gradient(90deg, #27ae60, #f39c12, #e74c3c); }
        .mem-fill { background: linear-gradient(90deg, #3498db, #2980b9); }
        .disk-fill { background: linear-gradient(90deg, #1abc9c, #16a085); }
        .temp { text-align: center; font-size: 3em; font-weight: bold; color: #e74c3c; }
        .uptime { text-align: center; font-size: 1.5em; color: #3498db; padding: 15px; }
        .time { text-align: center; color: #999; margin-top: 20px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🍓 树莓派系统监控</h1>
        <div class="grid">
            <div class="card">
                <h3>💻 CPU</h3>
                <div class="stat"><span class="label">使用率</span><span class="value" id="cpu">--%</span></div>
                <div class="bar"><div class="fill cpu-fill" id="cpuBar" style="width:0%"></div></div>
                <div class="temp" id="temp">--°C</div>
            </div>
            <div class="card">
                <h3>🧠 内存</h3>
                <div class="stat"><span class="label">总量</span><span class="value" id="memTotal">--GB</span></div>
                <div class="stat"><span class="label">已用</span><span class="value" id="memUsed">--GB</span></div>
                <div class="bar"><div class="fill mem-fill" id="memBar" style="width:0%"></div></div>
                <div class="stat"><span class="label">使用率</span><span class="value" id="memPct">--%</span></div>
            </div>
            <div class="card">
                <h3>💾 磁盘</h3>
                <div class="stat"><span class="label">总量</span><span class="value" id="diskTotal">--GB</span></div>
                <div class="stat"><span class="label">已用</span><span class="value" id="diskUsed">--GB</span></div>
                <div class="bar"><div class="fill disk-fill" id="diskBar" style="width:0%"></div></div>
                <div class="stat"><span class="label">使用率</span><span class="value" id="diskPct">--%</span></div>
            </div>
            <div class="card">
                <h3>⏱️ 运行时间</h3>
                <div class="uptime" id="uptime">加载中...</div>
            </div>
            <div class="card">
                <h3>📶 网络</h3>
                <div class="stat"><span class="label">上传</span><span class="value" id="sent">--</span></div>
                <div class="stat"><span class="label">下载</span><span class="value" id="recv">--</span></div>
            </div>
        </div>
        <div class="time">更新: <span id="time">--</span></div>
    </div>
    <script>
        function fmt(b){if(!b)return'0 B';const k=1024,s=['B','KB','MB','GB'];let i=Math.floor(Math.log(b)/Math.log(k));return(b/Math.pow(k,i)).toFixed(2)+' '+s[i];}
        function upd(){
            fetch('/api/sys').then(r=>r.json()).then(d=>{
                document.getElementById('cpu').innerText=d.cpu.usage.toFixed(1)+'%';
                document.getElementById('cpuBar').style.width=d.cpu.usage+'%';
                document.getElementById('temp').innerText=d.cpu.temp?d.cpu.temp.toFixed(1)+'°C':'N/A';
                document.getElementById('memTotal').innerText=d.memory.total+'GB';
                document.getElementById('memUsed').innerText=d.memory.used+'GB';
                document.getElementById('memBar').style.width=d.memory.percent+'%';
                document.getElementById('memPct').innerText=d.memory.percent.toFixed(1)+'%';
                document.getElementById('diskTotal').innerText=d.disk.total+'GB';
                document.getElementById('diskUsed').innerText=d.disk.used+'GB';
                document.getElementById('diskBar').style.width=d.disk.percent+'%';
                document.getElementById('diskPct').innerText=d.disk.percent.toFixed(1)+'%';
                document.getElementById('uptime').innerText=d.uptime;
                document.getElementById('sent').innerText=fmt(d.network.bytes_sent);
                document.getElementById('recv').innerText=fmt(d.network.bytes_recv);
                document.getElementById('time').innerText=d.timestamp;
            });
        }
        upd();setInterval(upd,3000);
    </script>
</body>
</html>
'''

def get_system_info():
    try:
        temps = psutil.sensors_temperatures()
        temp = None
        for k in temps:
            if 'cpu' in k.lower() or 'soc' in k.lower():
                temp = temps[k][0].current
                break
    except:
        temp = None
    
    return {
        'cpu': {'usage': psutil.cpu_percent(0.5), 'temp': temp},
        'memory': psutil.virtual_memory()._asdict(),
        'disk': psutil.disk_usage('/')._asdict(),
        'network': psutil.net_io_counters()._asdict(),
        'uptime': str(int(psutil.boot_time())) if psutil.boot_time() else '0',
        'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    }

def format_uptime():
    boot = psutil.boot_time()
    secs = datetime.now().timestamp() - boot
    d, h = int(secs // 86400), int((secs % 86400) // 3600)
    m = int((secs % 3600) // 60)
    if d: return f'{d}天 {h}小时'
    if h: return f'{h}小时 {m}分钟'
    return f'{m}分钟'

@app.route('/')
def index():
    d = get_system_info()
    d['uptime'] = format_uptime()
    return render_template_string(HTML)

@app.route('/api/sys')
def api():
    d = get_system_info()
    d['uptime'] = format_uptime()
    return jsonify(d)

if __name__ == '__main__':
    print('='*40)
    print('🌡️  树莓派轻量监控启动')
    print('访问: http://<ip>:5001')
    print('='*40)
    app.run(host='0.0.0.0', port=5001, debug=False)