# RaspClaws Pro - Pi Zero 2W & Ubuntu Server Guide

## System Requirements
- Raspberry Pi Zero 2W
- Ubuntu Server (Latest LTS version)
- Original Adeept RaspClaws Pro hardware components

## Initial Setup
1. Install Ubuntu Server on Pi Zero 2W
   ```bash
   # Download Ubuntu Server image for ARM64
   # Flash using Raspberry Pi Imager or similar tool
   ```

2. Required Package Installation
   ```bash
   sudo apt update
   ```

## Component Debugging Commands

### 1. Motor Control
```bash
# Test left motor
python3 -c "
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)  # Adjust pin numbers as needed
GPIO.output(17, GPIO.HIGH)
"

# Test right motor
# [Similar command with appropriate pin]
```

### 2. Servo Control
```bash
# Test pan servo
python3 -c "
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
servo = GPIO.PWM(18, 50)  # Adjust pin number
servo.start(7.5)
time.sleep(1)
servo.stop()
"
```

### 3. Camera Module
```bash
# Test camera
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
print(f'Camera working: {ret}')
cap.release()
"
```

### 4. Ultrasonic Sensor
```bash
# Test distance measurement
python3 -c "
import RPi.GPIO as GPIO
import time
TRIG = 23  # Adjust pin numbers
ECHO = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
# [Rest of testing code]
"
```

## Common Issues and Solutions

### 1. GPIO Permission Issues
```bash
# Add user to gpio group
sudo usermod -a -G gpio $USER
# Create udev rule if needed
sudo nano /etc/udev/rules.d/99-gpio.rules
# Add: SUBSYSTEM=="gpio", GROUP="gpio", MODE="0660"
```

### 2. Camera Access Issues
```bash
# Check camera module detection
vcgencmd get_camera  # If available on Ubuntu
ls -l /dev/video*
```

### 3. Motor Driver Issues
- Check voltage levels with multimeter
- Verify PWM signal generation
- Test motor driver board isolation

## Performance Optimization
1. Reduce CPU usage:
   ```bash
   # Monitor CPU usage
   top
   # Check temperature
   vcgencmd measure_temp
   ```

2. Network optimization:
   ```bash
   # Test network latency
   ping -c 4 8.8.8.8
   # Monitor bandwidth
   iftop
   ```

## Updating the Code
1. Update dependencies in requirements.txt
2. Modify pin configurations for Pi Zero 2W
3. Optimize video streaming for lower CPU usage
4. Add error handling and logging

## Testing Protocol
1. Individual component testing
2. Integration testing
3. Performance benchmarking
4. Network stability testing
