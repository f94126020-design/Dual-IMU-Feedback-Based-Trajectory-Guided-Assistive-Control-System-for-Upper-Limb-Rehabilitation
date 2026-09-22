#!/bin/bash
# Motor Control System - Startup Script
# 馬達控制系統 - 啟動腳本

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print colored messages
print_info() {
    echo -e "${BLUE}► ${1}${NC}"
}

print_success() {
    echo -e "${GREEN}✓ ${1}${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ ${1}${NC}"
}

print_error() {
    echo -e "${RED}✗ ${1}${NC}"
}

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

print_info "========================================="
print_info "  IMU Motor Control System Startup"
print_info "========================================="

# Check if running as root
if [[ $EUID -ne 0 && "$1" != "test" ]]; then
    print_warning "Not running as root. GPIO access may be limited."
    print_info "For GPIO access, run with: sudo $0"
fi

# Check if virtual environment exists
if [ ! -d ".venv" ]; then
    print_warning "Virtual environment not found. Creating one..."
    python3 -m venv .venv
fi

# Activate virtual environment
print_info "Activating virtual environment..."
source .venv/bin/activate
print_success "Virtual environment activated"

# Check and install requirements
print_info "Checking Python dependencies..."
if ! pip list | grep -q "Flask"; then
    print_warning "Installing requirements..."
    pip install -r requirements.txt
    print_success "Dependencies installed"
else
    print_success "All dependencies satisfied"
fi

# Check if app_motor_control.py exists
if [ ! -f "app_motor_control.py" ]; then
    print_error "app_motor_control.py not found!"
    exit 1
fi

# Show menu
print_info "========================================="
print_info "Select startup option:"
print_info "========================================="
echo "1) Start Motor Control System (Normal mode)"
echo "2) Start with Debug Output"
echo "3) Check Hardware Status"
echo "4) Run Tests"
echo "5) Exit"
echo ""
read -p "Enter choice [1-5]: " choice

case $choice in
    1)
        print_info "Starting Motor Control System..."
        python3 app_motor_control.py
        ;;
    2)
        print_info "Starting Motor Control System (Debug mode)..."
        PYTHONUNBUFFERED=1 python3 -u app_motor_control.py
        ;;
    3)
        print_info "Checking hardware status..."
        print_info ""
        print_info "Checking I2C devices..."
        if command -v i2cdetect &> /dev/null; then
            i2cdetect -y 1
        else
            print_warning "i2c-tools not installed. Run: sudo apt-get install i2c-tools"
        fi
        
        print_info ""
        print_info "Checking GPIO availability..."
        python3 << 'EOF'
try:
    import RPi.GPIO as GPIO
    print("✓ RPi.GPIO available")
    GPIO.setmode(GPIO.BCM)
    print("✓ GPIO mode set to BCM")
    GPIO.cleanup()
except ImportError:
    print("✗ RPi.GPIO not installed")
except Exception as e:
    print(f"✗ GPIO error: {e}")
EOF
        ;;
    4)
        print_info "Running tests..."
        python3 << 'EOF'
import sys
print("Testing imports...")

try:
    import smbus2
    print("✓ smbus2 available")
except ImportError:
    print("✗ smbus2 missing")
    sys.exit(1)

try:
    import RPi.GPIO
    print("✓ RPi.GPIO available")
except ImportError:
    print("⚠ RPi.GPIO not available (might be OK on non-Pi systems)")

try:
    import Flask
    print("✓ Flask available")
except ImportError:
    print("✗ Flask missing")
    sys.exit(1)

print("\nAll critical packages available!")
EOF
        ;;
    5)
        print_info "Exiting..."
        exit 0
        ;;
    *)
        print_error "Invalid choice"
        exit 1
        ;;
esac

print_success "Done"
