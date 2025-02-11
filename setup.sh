#!/bin/bash
# Adeept Rasptank Pro Robot Setup Script
#
# This script installs and configures system and Python packages for the robot,
# sets up I²C/SPI and GPIO, configures firmware settings (keeping audio enabled for TTS),
# and creates a startup script.
#
# The script uses individual sudo calls rather than requiring the entire script
# to run as root. It also includes error trapping to aid in debugging.
#
# Author: Alexander Gomez (2025-02-10)

# ------------------------------------------------------------------------------
# Debug / Error Handling Setup
# ------------------------------------------------------------------------------
# Exit on any unhandled error and print the line number and command that failed.
set -euo pipefail
trap 'echo "Error occurred at line ${LINENO}: $(sed -n "${LINENO}p" "$0")" >&2' ERR

# Define a helper function to run commands with a log message.
run_cmd() {
    echo ">> Running: $*"
    "$@"
    local status=$?
    if [ $status -ne 0 ]; then
         echo "Error: Command failed: $*" >&2
         exit $status
    fi
}

# Get the home directory of the non-root user running the script.
USER_HOME=$(eval echo "~$USER")
STARTUP_SCRIPT="$USER_HOME/startup.sh"

# ------------------------------------------------------------------------------
# 1. Update and Upgrade the System
# ------------------------------------------------------------------------------
echo "Updating package lists..."
for i in {1..3}; do
    if sudo apt update; then
         break
    else
         echo "Attempt $i: apt update failed. Retrying in 2 seconds..."
         sleep 2
    fi
done

echo "Upgrading installed packages..."
run_cmd sudo apt upgrade -y

# ------------------------------------------------------------------------------
# 2. Upgrade pip for Python3
# ------------------------------------------------------------------------------
echo "Upgrading pip..."
for i in {1..3}; do
    if sudo -H pip3 install --upgrade pip; then
         break
    else
         echo "Attempt $i: pip upgrade failed. Retrying in 2 seconds..."
         sleep 2
    fi
done

# ------------------------------------------------------------------------------
# 3. Install Development Tools and System Libraries
# ------------------------------------------------------------------------------
echo "Installing development packages and libraries..."
run_cmd sudo apt install -y python3-dev python3-pip python3-venv \
    libfreetype6-dev libjpeg-dev build-essential

# ------------------------------------------------------------------------------
# 4. Install Python Packages via apt
# ------------------------------------------------------------------------------
echo "Installing Python packages via apt..."
run_cmd sudo apt install -y \
    python3-flask \
    python3-rpi-ws281x \
    python3-flask-cors \
    python3-websockets \
    python3-numpy \
    python3-opencv \
    python3-smbus \
    python3-luma.oled

# ------------------------------------------------------------------------------
# 5. Install Additional Python Packages Using pip
# ------------------------------------------------------------------------------
echo "Installing additional Python packages via pip..."
run_cmd sudo -H pip3 install --break-system-packages \
    adafruit-pca9685 \
    mpu6050-raspberrypi \
    RPi.GPIO \
    adafruit-blinka \
    adafruit-circuitpython-pca9685 \
    imutils

# ------------------------------------------------------------------------------
# 6. Install Additional System Packages
# ------------------------------------------------------------------------------
echo "Installing additional system packages..."
run_cmd sudo apt install -y \
    i2c-tools \
    ffmpeg \
    v4l-utils \
    libcamera-tools \
    libcamera-v4l2 \
    libttspico-utils

# ------------------------------------------------------------------------------
# 7. Enable and Start SSH Service
# ------------------------------------------------------------------------------
echo "Enabling and starting SSH service..."
run_cmd sudo systemctl enable ssh
run_cmd sudo systemctl start ssh

# ------------------------------------------------------------------------------
# 8. Set Up I²C
# ------------------------------------------------------------------------------
echo "Configuring I²C..."
run_cmd sudo modprobe i2c-dev
if ! grep -q "^i2c-dev" /etc/modules; then
    echo "i2c-dev" | sudo tee -a /etc/modules
fi

# ------------------------------------------------------------------------------
# 9. Set Up GPIO Permissions
# ------------------------------------------------------------------------------
echo "Configuring GPIO permissions..."
# Add the current (non-root) user to the gpio group.
run_cmd sudo adduser "$USER" gpio
# Adjust permissions for /dev/gpiomem if it exists.
if [ -e /dev/gpiomem ]; then
    sudo chown root:gpio /dev/gpiomem || true
    sudo chmod g+rw /dev/gpiomem || true
fi

# ------------------------------------------------------------------------------
# 10. Clone the Adeept_RaspTankPro Repository
# ------------------------------------------------------------------------------
if [ ! -d "$USER_HOME/Adeept_RaspTankPro" ]; then
    echo "Cloning Adeept_RaspTankPro repository into $USER_HOME..."
    run_cmd git clone https://github.com/adeept/Adeept_RaspTankPro.git "$USER_HOME/Adeept_RaspTankPro"
fi

# ------------------------------------------------------------------------------
# 11. (Optional) Set Up Wi‑Fi Hotspot Tools (create_ap)
# ------------------------------------------------------------------------------
if ! command -v create_ap >/dev/null 2>&1; then
    echo "Installing create_ap for Wi‑Fi hotspot functionality..."
    run_cmd git clone https://github.com/oblique/create_ap /tmp/create_ap
    (cd /tmp/create_ap && run_cmd sudo make install)
fi

# ------------------------------------------------------------------------------
# 12. Configure Firmware Settings (config.txt)
# ------------------------------------------------------------------------------
echo "Configuring firmware settings..."
CONFIG_FILE=""
if [ -f /boot/firmware/config.txt ]; then
    CONFIG_FILE="/boot/firmware/config.txt"
elif [ -f /boot/config.txt ]; then
    CONFIG_FILE="/boot/config.txt"
fi

if [ -n "$CONFIG_FILE" ]; then
    echo "Using config file: $CONFIG_FILE"
    # Enable I²C if not already enabled.
    if ! grep -q "^dtparam=i2c_arm=on" "$CONFIG_FILE"; then
        echo "dtparam=i2c_arm=on" | sudo tee -a "$CONFIG_FILE"
    fi
    # Enable SPI if not already enabled.
    if ! grep -q "^dtparam=spi=on" "$CONFIG_FILE"; then
        echo "dtparam=spi=on" | sudo tee -a "$CONFIG_FILE"
    fi
    # Enable camera support.
    if ! grep -q "^start_x=1" "$CONFIG_FILE"; then
        echo "start_x=1" | sudo tee -a "$CONFIG_FILE"
    fi
    # **Keep Audio Enabled** for text-to-speech.
    if ! grep -q "^dtparam=audio=on" "$CONFIG_FILE"; then
        echo "dtparam=audio=on" | sudo tee -a "$CONFIG_FILE"
    fi
else
    echo "Warning: No config.txt found in /boot/firmware or /boot. Skipping firmware configuration."
fi

# ------------------------------------------------------------------------------
# 13. Create a Startup Script for Robot Control
# ------------------------------------------------------------------------------
echo "Creating startup script at $STARTUP_SCRIPT..."
cat <<EOF > "$STARTUP_SCRIPT"
#!/bin/bash
# Startup script for Adeept Rasptank Pro robot
# Uncomment one of the following commands to choose your control method

# Option 1: Launch the web server for remote control
sudo python3 \$HOME/Adeept_RaspTankPro/server/webServer.py

# Option 2: Launch the main server
# sudo python3 \$HOME/Adeept_RaspTankPro/server/server.py
EOF
run_cmd chmod +x "$STARTUP_SCRIPT"

# Optionally, add the startup script to /etc/rc.local if that file exists.
if [ -f /etc/rc.local ]; then
    if ! grep -q "$STARTUP_SCRIPT" /etc/rc.local; then
        echo "Adding startup script to /etc/rc.local..."
        sudo sed -i '/exit 0/d' /etc/rc.local
        echo "$STARTUP_SCRIPT &" | sudo tee -a /etc/rc.local
        echo "exit 0" | sudo tee -a /etc/rc.local
    fi
fi

# ------------------------------------------------------------------------------
# 14. Final Message
# ------------------------------------------------------------------------------
echo "Adeept Rasptank Pro Setup completed successfully!"
echo "Please review the output for any errors."
echo "When ready, you can reboot your system (sudo reboot) to apply all changes."

