#!/bin/bash

# Raspberry Pi IMU Monitor Quick Start Script

echo "╔════════════════════════════════════════════════════════════╗"
echo "║   Raspberry Pi IMU Monitor - Quick Start Script            ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Check if running on Raspberry Pi
if ! grep -q "Raspberry" /proc/device-tree/model 2>/dev/null; then
    echo "⚠  Warning: This script is designed for Raspberry Pi"
    echo "   It may still work on other Linux systems"
fi

echo ""
echo "Step 1: Checking Python installation..."
if ! command -v python3 &> /dev/null; then
    echo "✗ Python 3 not found. Please install Python 3."
    exit 1
fi
echo "✓ Python 3 found: $(python3 --version)"

echo ""
echo "Step 2: Creating virtual environment..."
if [ ! -d "venv" ]; then
    python3 -m venv venv
    echo "✓ Virtual environment created"
else
    echo "✓ Virtual environment already exists"
fi

echo ""
echo "Step 3: Activating virtual environment..."
source venv/bin/activate
echo "✓ Virtual environment activated"

echo ""
echo "Step 4: Installing dependencies..."
echo "   This may take a few minutes..."
pip install -q --upgrade pip
pip install -q -r requirements.txt

if [ $? -ne 0 ]; then
    echo "✗ Failed to install dependencies"
    exit 1
fi
echo "✓ Dependencies installed"

echo ""
echo "Step 5: Checking I2C connection..."
if command -v i2cdetect &> /dev/null; then
    echo ""
    echo "Scanning I2C bus 1 for MPU6050 (address 0x68)..."
    i2cdetect -y 1 | grep -q "68"
    
    if [ $? -eq 0 ]; then
        echo "✓ MPU6050 found at address 0x68!"
    else
        echo "⚠  MPU6050 not detected"
        echo "   Please check your connections"
        echo "   Application will start in simulation mode"
    fi
else
    echo "⚠  i2c-tools not installed. Skipping I2C detection."
    echo "   Install with: sudo apt-get install i2c-tools"
fi

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║                   Ready to start!                          ║"
echo "╠════════════════════════════════════════════════════════════╣"
echo "║                                                            ║"
echo "║  Your Raspberry Pi IP address is:                         ║"

IP_ADDR=$(hostname -I | awk '{print $1}')
echo "║  → $IP_ADDR"

echo "║                                                            ║"
echo "║  Access the web interface at:                             ║"
echo "║  → http://$IP_ADDR:5000"
echo "║                                                            ║"
echo "║  (Also accessible from this machine at http://localhost:5000)"
echo "║                                                            ║"
echo "╠════════════════════════════════════════════════════════════╣"
echo "║  Press Enter to start the application...                  ║"
echo "║  (Press Ctrl+C to cancel)                                 ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

read -p ""

echo ""
echo "🚀 Starting Raspberry Pi IMU Monitor..."
echo ""

python app.py
