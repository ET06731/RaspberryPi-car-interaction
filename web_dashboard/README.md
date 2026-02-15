# 小车Web仪表盘

通过局域网Web界面控制智能小车。

## 功能特性

- 🎮 **多种控制方式**: 按钮、键盘、触摸屏
- 📊 **实时数据显示**: 超声波距离、当前速度
- 🎚️ **速度调节**: 0-100% 无级调速
- 📱 **响应式设计**: 支持桌面和移动设备
- ⌨️ **键盘快捷键**: W/A/S/D/Q/E/空格

## 快速开始

### 1. 启动服务器

```bash
cd /home/yoi/rpi-car-project/web_dashboard
./start.sh
```

或使用Python直接启动：

```bash
cd /home/yoi/rpi-car-project/web_dashboard
sudo python3 app.py
```

### 2. 访问仪表盘

在同一局域网内的设备浏览器中访问：

```
http://<树莓派IP地址>:5000
```

查看树莓派IP地址：
```bash
hostname -I
```

## API接口

### 获取状态
```
GET /api/status
Response: {"speed": 80, "distance": 45.2}
```

### 控制移动
```
POST /api/control/move
Body: {"direction": "forward"}
方向: forward, backward, left, right, spin_left, spin_right, stop
```

### 设置速度
```
POST /api/control/speed
Body: {"speed": 80}
```

### 获取距离
```
GET /api/sensor/distance
Response: {"distance": 45.2, "unit": "cm"}
```

## 键盘快捷键

| 按键 | 功能 |
|------|------|
| W / ↑ | 前进 |
| S / ↓ | 后退 |
| A / ← | 左转 |
| D / → | 右转 |
| Q | 原地左转 |
| E | 原地右转 |
| 空格 | 停止 |

## 注意事项

1. **需要root权限**: GPIO操作需要sudo权限
2. **端口5000**: 确保端口未被占用
3. **防火墙**: 确保防火墙允许5000端口访问

## 文件结构

```
web_dashboard/
├── app.py              # Flask应用主文件
├── car_api.py          # 小车控制API
├── start.sh            # 启动脚本
├── requirements.txt    # Python依赖
├── static/
│   ├── css/
│   │   └── style.css   # 样式文件
│   └── js/
│       └── app.js      # 前端逻辑
└── templates/
    └── index.html      # 主页面
```
