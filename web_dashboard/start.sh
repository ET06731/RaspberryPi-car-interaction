#!/bin/bash
# 小车Web仪表盘启动脚本

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "================================"
echo "  小车Web仪表盘启动器"
echo "================================"

# 检查是否在树莓派上
if ! command -v python3 &> /dev/null; then
    echo "错误: 未找到 Python3"
    exit 1
fi

# 激活虚拟环境（如果存在）
if [ -d "$PROJECT_DIR/.venv" ]; then
    echo "正在激活虚拟环境..."
    source "$PROJECT_DIR/.venv/bin/activate"
fi

# 检查Flask是否安装
if ! python3 -c "import flask" 2>/dev/null; then
    echo "正在安装依赖..."
    pip install flask
fi

# 获取IP地址
IP_ADDRESS=$(hostname -I | awk '{print $1}')

echo ""
echo "启动Web服务器..."
echo "访问地址: http://${IP_ADDRESS}:5000"
echo "本地访问: http://localhost:5000"
echo ""
echo "按 Ctrl+C 停止服务器"
echo "================================"

# 进入web_dashboard目录并启动
cd "$SCRIPT_DIR"
sudo python3 app.py
