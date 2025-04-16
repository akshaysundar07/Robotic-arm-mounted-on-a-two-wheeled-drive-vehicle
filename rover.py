import time
import RPi.GPIO as GPIO
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from flask import Flask, render_template_string, jsonify
import threading
import spidev
import numpy as np

# Create Flask app
app = Flask(_name_)

# --------------------- Vibration Analysis Setup ---------------------
# SPI Setup
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 5000000
spi.mode = 0b11

# ADXL345 Registers
REG_POWER_CTL = 0x2D
REG_DATAX0 = 0x32

# Constants
SAMPLES = 128
SAMPLING_RATE = 100  # Hz
INTERVAL = 1.0 / SAMPLING_RATE
THRESHOLD = 2000  # Threshold for anomaly detection
MIN_FREQ = 5  # Ignore DC/low frequency noise

# Global variables for sharing state between threads
anomaly_detected = False
peak_magnitude = 0
anomaly_timestamp = 0

# Init ADXL345
def adxl345_init():
    spi.xfer2([REG_POWER_CTL, 0x08])

def read_axis_z():
    resp = spi.xfer2([0x80 | 0x40 | (REG_DATAX0 + 4), 0x00, 0x00])
    value = resp[1] | (resp[2] << 8)
    if value & (1 << 15):
        value -= (1 << 16)
    return value

# Function to run vibration analysis in a separate thread
def vibration_analysis():
    global anomaly_detected, peak_magnitude, anomaly_timestamp
    
    adxl345_init()
    z_data = []
    
    while True:
        z_data.append(read_axis_z())
        if len(z_data) >= SAMPLES:
            z_array = np.array(z_data[-SAMPLES:])
            fft_vals = np.fft.fft(z_array)
            freqs = np.fft.fftfreq(SAMPLES, d=INTERVAL)
            magnitude = np.abs(fft_vals)[:SAMPLES // 2]
            freqs = freqs[:SAMPLES // 2]
            
            # Filter valid frequencies
            valid_indices = np.where(freqs > MIN_FREQ)
            filtered_magnitude = magnitude[valid_indices]
            
            # Anomaly detection
            if len(filtered_magnitude) > 0:
                current_peak_mag = np.max(filtered_magnitude)
                peak_magnitude = int(current_peak_mag)
                
                if current_peak_mag > THRESHOLD:
                    print(f"[ALERT] Anomaly Detected! Peak Magnitude: {peak_magnitude}")
                    anomaly_detected = True
                    anomaly_timestamp = time.time()
                else:
                    # Clear anomaly after 5 seconds
                    if anomaly_detected and time.time() - anomaly_timestamp > 5:
                        anomaly_detected = False
            
            # Remove oldest data to keep buffer size consistent
            z_data = z_data[-SAMPLES:]
            
        time.sleep(INTERVAL)

# --------------------- L298N Motor Driver Setup ---------------------
# Motor GPIO pin definitions
in1, in2, in3, in4 = 17, 27, 22, 23
ena, enb = 18, 13

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for pin in [in1, in2, in3, in4, ena, enb]:
    GPIO.setup(pin, GPIO.OUT)

# PWM setup
pwm_ena = GPIO.PWM(ena, 1000)
pwm_enb = GPIO.PWM(enb, 1000)
pwm_ena.start(60)
pwm_enb.start(60)

# Car movement functions
def car_forward():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def car_backward():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)

def car_left():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def car_right():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)

def car_stop():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)

# --------------------- PCA9685 Servo Setup ---------------------
# Initialize I2C and PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Servo channel assignments
base = servo.Servo(pca.channels[0])
gripper = servo.Servo(pca.channels[1])
elbow = servo.Servo(pca.channels[2])

# Initial angles
base_angle = 90
gripper_angle = 0
elbow_angle = 90

# Smooth angle movement function
def smooth_set_angle(servo_motor, current_angle, target_angle, speed=0.01):
    step = 1 if target_angle > current_angle else -1
    for angle in range(current_angle, target_angle + step, step):
        servo_motor.angle = angle
        time.sleep(speed)
    return target_angle

# --------------------- Flask Routes ---------------------
@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/control/<command>', methods=['GET'])
def control(command):
    global base_angle, gripper_angle, elbow_angle
    
    # Motor speed control
    if command.startswith('speed-'):
        speed = int(command.split('-')[1])
        pwm_ena.ChangeDutyCycle(speed)
        pwm_enb.ChangeDutyCycle(speed)
        return jsonify({'status': 'success', 'command': command, 'speed': speed})

    # Car movement
    if command == 'w':
        car_forward()
    elif command == 's':
        car_backward()
    elif command == 'a':
        car_left()
    elif command == 'd':
        car_right()
    elif command == 'stop':
        car_stop()
    
    # Arm movement
    elif command == 'i':
        new_angle = min(180, elbow_angle + 5)
        elbow_angle = smooth_set_angle(elbow, elbow_angle, new_angle)
    elif command == 'k':
        new_angle = max(0, elbow_angle - 5)
        elbow_angle = smooth_set_angle(elbow, elbow_angle, new_angle)
    elif command == 'j':
        new_angle = max(0, base_angle - 5)
        base_angle = smooth_set_angle(base, base_angle, new_angle)
    elif command == 'l':
        new_angle = min(180, base_angle + 5)
        base_angle = smooth_set_angle(base, base_angle, new_angle)
    elif command == 'o':
        new_angle = max(0, gripper_angle - 5)
        gripper_angle = smooth_set_angle(gripper, gripper_angle, new_angle)
    elif command == 'c':
        new_angle = min(90, gripper_angle + 5)
        gripper_angle = smooth_set_angle(gripper, gripper_angle, new_angle)

    # Return positions for the UI to update
    return jsonify({
        'status': 'success', 
        'command': command,
        'positions': {
            'base': base_angle,
            'elbow': elbow_angle,
            'gripper': gripper_angle
        }
    })

# Route to check for anomalies
@app.route('/check_anomaly', methods=['GET'])
def check_anomaly():
    global anomaly_detected, peak_magnitude
    return jsonify({
        'anomaly_detected': anomaly_detected,
        'peak_magnitude': peak_magnitude
    })

# --------------------- HTML Content ---------------------
HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Control Center</title>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
    <style>
        :root {
            --primary-color: #3498db;
            --secondary-color: #2ecc71;
            --accent-color: #e74c3c;
            --alert-color: #ff0000;
            --bg-color: #f9f9f9;
            --card-bg: #ffffff;
            --text-color: #333333;
            --border-radius: 10px;
            --box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }

        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background-color: var(--bg-color);
            color: var(--text-color);
            line-height: 1.6;
            padding: 20px;
            max-width: 1200px;
            margin: 0 auto;
            transition: background-color 0.5s ease;
        }

        body.alert {
            background-color: #ffdddd;
        }

        header {
            text-align: center;
            padding: 20px 0;
            margin-bottom: 30px;
            border-bottom: 2px solid var(--primary-color);
        }

        h1 {
            color: var(--primary-color);
            font-size: 2.5rem;
            margin-bottom: 10px;
        }

        .subtitle {
            font-size: 1.2rem;
            color: #666;
            margin-bottom: 20px;
        }

        .dashboard {
            display: grid;
            grid-template-columns: 1fr;
            gap: 30px;
        }

        @media (min-width: 768px) {
            .dashboard {
                grid-template-columns: 1fr 1fr;
            }
        }

        .control-panel {
            background-color: var(--card-bg);
            border-radius: var(--border-radius);
            padding: 20px;
            box-shadow: var(--box-shadow);
        }

        .panel-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 20px;
            padding-bottom: 10px;
            border-bottom: 1px solid #eee;
        }

        .panel-title {
            font-size: 1.5rem;
            color: var(--primary-color);
            display: flex;
            align-items: center;
            gap: 10px;
        }

        .control-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
        }

        .button {
            background-color: var(--primary-color);
            color: white;
            border: none;
            padding: 15px;
            font-size: 18px;
            border-radius: var(--border-radius);
            cursor: pointer;
            transition: all 0.3s ease;
            display: flex;
            justify-content: center;
            align-items: center;
            touch-action: manipulation;
            user-select: none;
        }

        .button:hover {
            transform: translateY(-2px);
            box-shadow: 0 6px 12px rgba(0, 0, 0, 0.1);
        }

        .button:active {
            transform: translateY(1px);
            box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
        }

        .button.primary {
            background-color: var(--primary-color);
        }

        .button.secondary {
            background-color: var(--secondary-color);
        }

        .button.accent {
            background-color: var(--accent-color);
        }

        .button.disabled {
            background-color: #ccc;
            cursor: not-allowed;
        }

        .button-col {
            display: flex;
            flex-direction: column;
            gap: 10px;
        }

        .keyboard-control {
            margin-top: 20px;
            padding: 15px;
            background-color: #f5f5f5;
            border-radius: var(--border-radius);
            text-align: center;
        }

        .keyboard-guide {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 15px;
            margin-top: 15px;
        }

        .key-item {
            display: flex;
            align-items: center;
            gap: 10px;
        }

        .key {
            display: inline-block;
            padding: 5px 10px;
            background-color: #eee;
            border-radius: 4px;
            border: 1px solid #ddd;
            font-weight: bold;
            min-width: 30px;
            text-align: center;
        }

        .slider-container {
            margin: 20px 0;
        }

        .slider-label {
            display: flex;
            justify-content: space-between;
            margin-bottom: 10px;
        }

        .slider {
            width: 100%;
            height: 10px;
            -webkit-appearance: none;
            appearance: none;
            background: #ddd;
            outline: none;
            border-radius: 5px;
        }

        .slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 25px;
            height: 25px;
            border-radius: 50%;
            background: var(--primary-color);
            cursor: pointer;
        }

        .slider::-moz-range-thumb {
            width: 25px;
            height: 25px;
            border-radius: 50%;
            background: var(--primary-color);
            cursor: pointer;
        }

        .status-display {
            background-color: #f0f8ff;
            border-radius: var(--border-radius);
            padding: 15px;
            margin-top: 20px;
        }

        .status-row {
            display: flex;
            justify-content: space-between;
            margin-bottom: 10px;
        }

        .status-label {
            font-weight: bold;
        }

        .visual-indicator {
            position: relative;
            height: 20px;
            background-color: #eee;
            border-radius: 10px;
            overflow: hidden;
        }

        .indicator-fill {
            position: absolute;
            height: 100%;
            background-color: var(--primary-color);
            border-radius: 10px;
            transition: width 0.3s ease;
        }

        .stop-button {
            background-color: var(--accent-color);
            color: white;
            border: none;
            padding: 15px;
            font-size: 18px;
            border-radius: var(--border-radius);
            cursor: pointer;
            width: 100%;
            margin-top: 20px;
            transition: all 0.3s ease;
            display: flex;
            justify-content: center;
            align-items: center;
            gap: 10px;
        }

        .stop-button:hover {
            background-color: #c0392b;
        }

        .anomaly-alert {
            display: none;
            background-color: var(--alert-color);
            color: white;
            text-align: center;
            padding: 15px;
            margin-bottom: 20px;
            border-radius: var(--border-radius);
            animation: pulse 1.5s infinite;
        }

        @keyframes pulse {
            0% { opacity: 1; }
            50% { opacity: 0.7; }
            100% { opacity: 1; }
        }

        .anomaly-alert.active {
            display: block;
        }

        @media (hover: none) {
            .button {
                padding: 20px;
                font-size: 22px;
            }
        }

        @media (prefers-color-scheme: dark) {
            :root {
                --bg-color: #121212;
                --card-bg: #1e1e1e;
                --text-color: #f1f1f1;
            }
            
            .keyboard-control {
                background-color: #2a2a2a;
            }
            
            .key {
                background-color: #333;
                border-color: #444;
                color: #eee;
            }
            
            .status-display {
                background-color: #2a2a2a;
            }
            
            body.alert {
                background-color: #3a0d0d;
            }
        }
    </style>
</head>
<body>
    <header>
        <h1>Robot Control Center</h1>
        <p class="subtitle">Remote interface for robot car and robotic arm operations</p>
    </header>

    <div class="anomaly-alert" id="anomalyAlert">
        <i class="fas fa-exclamation-triangle"></i>
        <strong>ANOMALY DETECTED! Vibration Level: <span id="peakMagnitude">0</span></strong>
    </div>

    <main class="dashboard">
        <section class="control-panel">
            <div class="panel-header">
                <h2 class="panel-title"><i class="fas fa-car"></i> Car Movement</h2>
            </div>
            
            <div class="slider-container">
                <div class="slider-label">
                    <span>Speed Control</span>
                    <span id="speedValue">60%</span>
                </div>
                <input type="range" min="0" max="100" value="60" class="slider" id="speedSlider">
            </div>

            <div class="control-grid">
                <div class="button disabled"></div>
                <button class="button primary" id="forward" onmousedown="startCommand('w')" onmouseup="stopCar()" ontouchstart="startCommand('w')" ontouchend="stopCar()">
                    <i class="fas fa-arrow-up"></i>
                </button>
                <div class="button disabled"></div>
                
                <button class="button primary" id="left" onmousedown="startCommand('a')" onmouseup="stopCar()" ontouchstart="startCommand('a')" ontouchend="stopCar()">
                    <i class="fas fa-arrow-left"></i>
                </button>
                <button class="button accent" onclick="sendCommand('stop')">
                    <i class="fas fa-stop"></i>
                </button>
                <button class="button primary" id="right" onmousedown="startCommand('d')" onmouseup="stopCar()" ontouchstart="startCommand('d')" ontouchend="stopCar()">
                    <i class="fas fa-arrow-right"></i>
                </button>
                
                <div class="button disabled"></div>
                <button class="button primary" id="backward" onmousedown="startCommand('s')" onmouseup="stopCar()" ontouchstart="startCommand('s')" ontouchend="stopCar()">
                    <i class="fas fa-arrow-down"></i>
                </button>
                <div class="button disabled"></div>
            </div>

            <div class="keyboard-control">
                <h3>Keyboard Controls</h3>
                <div class="keyboard-guide">
                    <div class="key-item">
                        <span class="key">W</span>
                        <span>Forward</span>
                    </div>
                    <div class="key-item">
                        <span class="key">S</span>
                        <span>Backward</span>
                    </div>
                    <div class="key-item">
                        <span class="key">A</span>
                        <span>Left</span>
                    </div>
                    <div class="key-item">
                        <span class="key">D</span>
                        <span>Right</span>
                    </div>
                    <div class="key-item">
                        <span class="key">Space</span>
                        <span>Stop</span>
                    </div>
                </div>
            </div>
        </section>

        <section class="control-panel">
            <div class="panel-header">
                <h2 class="panel-title"><i class="fas fa-robot"></i> Robotic Arm</h2>
            </div>
            
            <div class="control-grid">
                <div class="button-col">
                    <h3 class="panel-subtitle">Base</h3>
                    <button class="button secondary" id="base-left" onmousedown="startCommand('j')" onmouseup="stopCommand()" ontouchstart="startCommand('j')" ontouchend="stopCommand()">
                        <i class="fas fa-undo"></i>
                    </button>
                    <button class="button secondary" id="base-right" onmousedown="startCommand('l')" onmouseup="stopCommand()" ontouchstart="startCommand('l')" ontouchend="stopCommand()">
                        <i class="fas fa-redo"></i>
                    </button>
                </div>
                
                <div class="button-col">
                    <h3 class="panel-subtitle">Elbow</h3>
                    <button class="button secondary" id="elbow-up" onmousedown="startCommand('i')" onmouseup="stopCommand()" ontouchstart="startCommand('i')" ontouchend="stopCommand()">
                        <i class="fas fa-arrow-up"></i>
                    </button>
                    <button class="button secondary" id="elbow-down" onmousedown="startCommand('k')" onmouseup="stopCommand()" ontouchstart="startCommand('k')" ontouchend="stopCommand()">
                        <i class="fas fa-arrow-down"></i>
                    </button>
                </div>
                
                <div class="button-col">
                    <h3 class="panel-subtitle">Gripper</h3>
                    <button class="button secondary" id="gripper-open" onmousedown="startCommand('o')" onmouseup="stopCommand()" ontouchstart="startCommand('o')" ontouchend="stopCommand()">
                        <i class="fas fa-hand-scissors"></i> Open
                    </button>
                    <button class="button secondary" id="gripper-close" onmousedown="startCommand('c')" onmouseup="stopCommand()" ontouchstart="startCommand('c')" ontouchend="stopCommand()">
                        <i class="fas fa-fist-raised"></i> Close
                    </button>
                </div>
            </div>

            <div class="status-display">
                <h3>Arm Position</h3>
                
                <div class="status-row">
                    <span class="status-label">Base:</span>
                    <span id="base-position">90°</span>
                </div>
                <div class="visual-indicator">
                    <div class="indicator-fill" id="base-indicator" style="width: 50%;"></div>
                </div>
                
                <div class="status-row">
                    <span class="status-label">Elbow:</span>
                    <span id="elbow-position">90°</span>
                </div>
                <div class="visual-indicator">
                    <div class="indicator-fill" id="elbow-indicator" style="width: 50%;"></div>
                </div>
                
                <div class="status-row">
                    <span class="status-label">Gripper:</span>
                    <span id="gripper-position">0°</span>
                </div>
                <div class="visual-indicator">
                    <div class="indicator-fill" id="gripper-indicator" style="width: 0%;"></div>
                </div>
            </div>

            <div class="keyboard-control">
                <h3>Keyboard Controls</h3>
                <div class="keyboard-guide">
                    <div class="key-item">
                        <span class="key">J</span>
                        <span>Base Left</span>
                    </div>
                    <div class="key-item">
                        <span class="key">L</span>
                        <span>Base Right</span>
                    </div>
                    <div class="key-item">
                        <span class="key">I</span>
                        <span>Elbow Up</span>
                    </div>
                    <div class="key-item">
                        <span class="key">K</span>
                        <span>Elbow Down</span>
                    </div>
                    <div class="key-item">
                        <span class="key">O</span>
                        <span>Open Gripper</span>
                    </div>
                    <div class="key-item">
                        <span class="key">C</span>
                        <span>Close Gripper</span>
                    </div>
                </div>
            </div>
        </section>
    </main>

    <button class="stop-button" onclick="emergencyStop()">
        <i class="fas fa-exclamation-triangle"></i> EMERGENCY STOP
    </button>

    <script>
        const raspberryPiIP = window.location.origin;
        let commandInterval = null;
        let currentCommand = null;
        let anomalyCheckInterval = null;

        let armPositions = {
            base: 90,
            elbow: 90,
            gripper: 0
        };

        function sendCommand(command) {
            fetch(${raspberryPiIP}/control/${command}, {
                method: 'GET',
            })
            .then(response => response.json())
            .then(data => {
                console.log(data);
                if (data.positions) {
                    updateArmPositions(data.positions);
                }
            })
            .catch(error => console.error('Error:', error));
        }

        function startCommand(command) {
            if (commandInterval) {
                clearInterval(commandInterval);
            }
            currentCommand = command;
            sendCommand(command);
            commandInterval = setInterval(() => {
                sendCommand(command);
            }, 200);
        }

        function stopCommand() {
            if (commandInterval) {
                clearInterval(commandInterval);
                commandInterval = null;
            }
            currentCommand = null;
        }

        function stopCar() {
            stopCommand();
            sendCommand('stop');
        }

        function emergencyStop() {
            stopCommand();
            sendCommand('stop');
            document.body.style.backgroundColor = "#ffdddd";
            setTimeout(() => {
                document.body.style.backgroundColor = "";
            }, 500);
        }

        function updateArmPositions(positions) {
            armPositions = positions;
            document.getElementById("base-position").textContent = ${positions.base}°;
            document.getElementById("elbow-position").textContent = ${positions.elbow}°;
            document.getElementById("gripper-position").textContent = ${positions.gripper}°;
            document.getElementById("base-indicator").style.width = ${(positions.base / 180) * 100}%;
            document.getElementById("elbow-indicator").style.width = ${(positions.elbow / 180) * 100}%;
            document.getElementById("gripper-indicator").style.width = ${(positions.gripper / 90) * 100}%;
        }

        function checkForAnomalies() {
            fetch(${raspberryPiIP}/check_anomaly, {
                method: 'GET',
            })
            .then(response => response.json())
            .then(data => {
                if (data.anomaly_detected) {
                    document.getElementById("anomalyAlert").classList.add("active");
                    document.getElementById("peakMagnitude").textContent = data.peak_magnitude;
                    document.body.classList.add("alert");
                } else {
                    document.getElementById("anomalyAlert").classList.remove("active");
                    document.body.classList.remove("alert");
                }
            })
            .catch(error => console.error('Error checking anomalies:', error));
        }

        const speedSlider = document.getElementById("speedSlider");
        const speedValue = document.getElementById("speedValue");
        
        speedSlider.oninput = function() {
            speedValue.textContent = this.value + "%";
            sendCommand(speed-${this.value});
        }

        document.addEventListener('keydown', function(event) {
            if (!currentCommand) {
                switch(event.key.toLowerCase()) {
                    case 'w': startCommand('w'); break;
                    case 's': startCommand('s'); break;
                    case 'a': startCommand('a'); break;
                    case 'd': startCommand('d'); break;
                    case ' ': stopCar(); break;
                    case 'i': startCommand('i'); break;
                    case 'k': startCommand('k'); break;
                    case 'j': startCommand('j'); break;
                    case 'l': startCommand('l'); break;
                    case 'o': startCommand('o'); break;
                    case 'c': startCommand('c'); break;
                }
            }
        });

        document.addEventListener('keyup', function(event) {
            if (currentCommand) {
                const key = event.key.toLowerCase();
                if ((key === 'w' && currentCommand === 'w') ||
                    (key === 's' && currentCommand === 's') ||
                    (key === 'a' && currentCommand === 'a') ||
                    (key === 'd' && currentCommand === 'd') ||
                    (key === 'i' && currentCommand === 'i') ||
                    (key === 'k' && currentCommand === 'k') ||
                    (key === 'j' && currentCommand === 'j') ||
                    (key === 'l' && currentCommand === 'l') ||
                    (key === 'o' && currentCommand === 'o') ||
                    (key === 'c' && currentCommand === 'c')) {
                    if (['w', 's', 'a', 'd'].includes(key)) {
                        stopCar();
                    } else {
                        stopCommand();
                    }
                }
            }
        });

        anomalyCheckInterval = setInterval(checkForAnomalies, 1000);
        checkForAnomalies();
    </script>
</body>
</html>
"""

# --------------------- Main Application Entry ---------------------
if _name_ == "_main_":
    try:
        # Start the vibration analysis in a separate thread
        vibration_thread = threading.Thread(target=vibration_analysis, daemon=True)
        vibration_thread.start()
        
        # Initialize servo positions
        base.angle = base_angle
        elbow.angle = elbow_angle
        gripper.angle = gripper_angle
        
        # Start Flask server
        app.run(host='0.0.0.0', port=5000, debug=False)
    finally:
        # Clean up GPIO on exit
        GPIO.cleanup()
        pca.deinit()
        spi.close()
