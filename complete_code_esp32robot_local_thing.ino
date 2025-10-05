#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>
#include <ESPmDNS.h> // NEW: Include MDNS library for hostname resolution

// --- Wi-Fi Credentials ---
const char *ssid = "mycar";
const char *password = "12345678";

// --- MDNS Hostname ---
const char* host = "esp32robot"; // Access device via http://esp32robot.local

// --- MOTOR DRIVER PINS (L298N) ---
#define MOTOR_EN_PIN    14
#define R_FORWARD_PIN   33
#define R_BACKWARD_PIN  32
#define L_FORWARD_PIN   26
#define L_BACKWARD_PIN  27

// --- SCANNING SERVO PINS (Positional Servos) ---
const int panServoPin = 23;  // Horizontal movement (Servo 1)
const int tiltServoPin = 22; // Vertical movement (Servo 2, now on GPIO 22)
Servo servoPan;
Servo servoTilt;

// --- ULTRASONIC PINS ---
const int trigPin = 5;  // Trigger Pin
const int echoPin = 18; // Echo Pin

// --- LASER RELAY PIN ---
const int laserPin = 19; 

// --- SCANNING/CONTROL PARAMETERS ---
const int panStart = 0;
const int panEnd = 180;
const int panStep = 5; // Reduced step for smoother sweep

const int tiltStart = 70; // AUTO SWEEP MIN TILT
const int tiltEnd = 120;  // AUTO SWEEP MAX TILT
const int tiltStep = 10;

const int servoSettleTime = 100; // Time to allow servo to reach new position
const int SHOOT_DISTANCE = 100;  // Detection threshold in cm (1 meter)
const int RADAR_RANGE = 100;     // Radar visualization limit in cm (1 meter)
const int SINGLE_SHOT_TIME = 100;// Time in ms for auto-mode pulse fire

// --- STATE MANAGEMENT ---
bool autoScanMode = true; // true = Auto Scan, false = Manual Control
int currentPanAngle = panStart;
int currentTiltAngle = tiltStart;
bool panDirection = true;  // true = 0->180, false = 180->0
bool tiltDirection = true; // true = 70->120 (up), false = 120->70 (down) <--- NEW STATE

// --- CONTINUOUS FIRE STATE (MACHINE GUN) ---
bool isManualFiring = false;
unsigned long lastLaserToggleTime = 0;
const int laserToggleInterval = 50; // 50ms on/off pulse (20Hz)

// --- WEB SERVER SETUP ---
WebServer server(80);
WebSocketsServer webSocket(81);


// ====================================================================
//                            HTML CONTENT (UPDATED LAYOUT)
// ====================================================================

const char* HTML_CONTENT = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Targeter Robot</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
        body { font-family: 'Inter', sans-serif; background-color: #1f2937; color: #f9fafb; }
        .card { 
            background-color: #374151; 
            border-radius: 1rem; 
            padding: 1.5rem; 
            box-shadow: 0 8px 15px rgba(0, 0, 0, 0.2); 
        }
        .btn { 
            padding: 0.75rem 1.5rem; 
            border-radius: 0.5rem; 
            font-weight: 600; 
            transition: all 0.1s; 
            box-shadow: 0 4px #111827;
            border-bottom: 2px solid #111827;
        }
        .drive-btn { background-color: #10b981; color: white; }
        .drive-btn:active { background-color: #059669; transform: translateY(2px) scale(0.98); box-shadow: 0 2px #111827; }
        .fire-btn { background-color: #ef4444; color: white; }
        .fire-btn:active { background-color: #dc2626; transform: translateY(2px) scale(0.98); box-shadow: 0 2px #111827; }
        .disabled-input { opacity: 0.5; pointer-events: none; }
        input[type=range] {
            -webkit-appearance: none;
            width: 100%;
            height: 10px;
            background: #5c6776;
            border-radius: 5px;
            outline: none;
            opacity: 0.9;
            transition: opacity .15s;
        }
        input[type=range]::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: #6366f1;
            cursor: pointer;
        }
        /* Custom Radar CSS */
        #radarCanvas { border-radius: 1rem; background-color: #111827; }
        #radarContainer { display: flex; flex-direction: column; align-items: center; }
        #distanceDisplay {
            margin-top: 1rem;
            background: rgba(55, 65, 81, 0.9);
            padding: 0.5rem 1rem;
            border-radius: 0.5rem;
            font-size: 1.1rem;
            font-weight: bold;
        }
    </style>
</head>
<body class="p-4 md:p-8">
    <div class="max-w-7xl mx-auto space-y-6">
        <h1 class="text-3xl font-bold text-center text-indigo-400">ESP32 Combat Robot Control</h1>
        
        <!-- TOP ROW: CONTROLS -->
        <div class="grid grid-cols-1 lg:grid-cols-2 gap-6">

            <!-- 1. DRIVING CONTROLS (Left Panel) -->
            <div id="driveControls" class="card flex flex-col items-center justify-center">
                <h2 class="text-xl font-semibold mb-6 text-indigo-300">Vehicle Drive (Hold to Move)</h2>
                <div class="grid grid-cols-3 gap-4">
                    <div class="col-span-3 flex justify-center">
                        <button class="btn drive-btn w-32 h-16" ontouchstart="sendCmd('F_START')" ontouchend="sendCmd('S_STOP')" onmousedown="sendCmd('F_START')" onmouseup="sendCmd('S_STOP')">FORWARD</button>
                    </div>
                    <button class="btn drive-btn w-32 h-16" ontouchstart="sendCmd('L_START')" ontouchend="sendCmd('S_STOP')" onmousedown="sendCmd('L_START')" onmouseup="sendCmd('S_STOP')">LEFT</button>
                    <button class="btn drive-btn w-32 h-16 bg-red-500 hover:bg-red-600" ontouchstart="sendCmd('S_STOP')" ontouchend="sendCmd('S_STOP')" onmousedown="sendCmd('S_STOP')" onmouseup="sendCmd('S_STOP')">STOP</button>
                    <button class="btn drive-btn w-32 h-16" ontouchstart="sendCmd('R_START')" ontouchend="sendCmd('S_STOP')" onmousedown="sendCmd('R_START')" onmouseup="sendCmd('S_STOP')">RIGHT</button>
                    <div class="col-span-3 flex justify-center">
                        <button class="btn drive-btn w-32 h-16" ontouchstart="sendCmd('B_START')" ontouchend="sendCmd('S_STOP')" onmousedown="sendCmd('B_START')" onmouseup="sendCmd('S_STOP')">BACKWARD</button>
                    </div>
                </div>
            </div>

            <!-- 2. SERVO & LASER CONTROLS (Right Panel) -->
            <div id="servoControls" class="card space-y-4">
                <h2 class="text-xl font-semibold text-indigo-300 mb-6">Turret & Firing Control</h2>

                <!-- Mode Toggle -->
                <div class="flex justify-between items-center bg-gray-600 p-3 rounded-lg">
                    <span class="font-medium">Control Mode:</span>
                    <label class="relative inline-flex items-center cursor-pointer">
                        <input type="checkbox" id="modeToggle" class="sr-only peer" checked onchange="toggleMode(this)">
                        <div class="w-11 h-6 bg-gray-200 rounded-full peer peer-focus:ring-4 peer-focus:ring-indigo-300 dark:peer-focus:ring-indigo-800 dark:bg-gray-700 peer-checked:after:translate-x-full peer-checked:after:border-white after:content-[''] after:absolute after:top-0.5 after:left-[2px] after:bg-white after:border-gray-300 after:border after:rounded-full after:h-5 after:w-5 after:transition-all dark:border-gray-600 peer-checked:bg-indigo-600"></div>
                        <span id="modeText" class="ml-3 text-sm font-medium text-gray-900 dark:text-gray-300">AUTO SWEEP</span>
                    </label>
                </div>

                <!-- Manual Sliders (Disabled by default) -->
                <div id="manualSliders" class="disabled-input space-y-4">
                    <label class="block font-medium">Manual Pan (<span id="panValue">0</span>&deg;):</label>
                    <input type="range" min="0" max="180" value="0" id="panSlider" oninput="updateServo('PAN', this.value)">

                    <!-- UPDATED: Tilt range changed from 70-120 to 0-180 -->
                    <label class="block font-medium">Manual Tilt (<span id="tiltValue">90</span>&deg;):</label>
                    <input type="range" min="0" max="180" value="90" id="tiltSlider" oninput="updateServo('TILT', this.value)">
                </div>

                <!-- Auto Display (Enabled by default) -->
                <div id="autoDisplay" class="space-y-4">
                    <p class="font-medium">Auto Pan: <span id="autoPanValue" class="font-bold text-yellow-300">0</span>&deg;</p>
                    <p class="font-medium">Auto Tilt: <span id="autoTiltValue" class="font-bold text-yellow-300">70</span>&deg;</p>
                </div>

                <!-- Fire Button - Updated for continuous fire (Hold-to-Fire) -->
                <button class="btn fire-btn w-full mt-6" 
                    ontouchstart="sendCmd('FIRE_START')" ontouchend="sendCmd('FIRE_STOP')" 
                    onmousedown="sendCmd('FIRE_START')" onmouseup="sendCmd('FIRE_STOP')">
                    <svg xmlns="http://www.w3.org/2000/svg" class="h-6 w-6 inline mr-2" viewBox="0 0 20 20" fill="currentColor">
                        <path fill-rule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM9.555 7.168A1 1 0 008 8v4a1 1 0 001.555.832l3-2a1 1 0 000-1.664l-3-2z" clip-rule="evenodd" />
                    </svg>
                    MANUAL FIRE (Machine Gun)
                </button>
            </div>
        </div>

        <!-- BOTTOM ROW: RADAR DISPLAY -->
        <div id="radarContainer" class="card">
            <h2 class="text-xl font-semibold mb-4 text-center text-indigo-300">Detection Radar (1m Range)</h2>
            <div class="flex justify-center relative">
                <canvas id="radarCanvas" width="500" height="250" class="mx-auto"></canvas>
                <div id="distanceDisplay" class="text-white absolute top-4 right-4 lg:right-auto lg:left-4">Distance: <span id="currentDistance">---</span> cm</div>
            </div>
        </div>
        
    </div>

<script>
    let ws;
    let autoScanActive = true;
    const RADAR_RANGE_CM = 100; // 1 meter for visualization
    const RADAR_CANVAS = document.getElementById('radarCanvas');
    const CTX = RADAR_CANVAS.getContext('2d');
    const CENTER_X = RADAR_CANVAS.width / 2;
    const CENTER_Y = RADAR_CANVAS.height;
    const MAX_RADIUS = RADAR_CANVAS.height;

    // --- WebSocket Communication ---
    function connectWebSocket() {
        // Use location.hostname, which will automatically resolve to the mDNS hostname (esp32robot.local)
        // or the IP address, depending on how the page was loaded.
        const hostname = location.hostname;
        ws = new WebSocket('ws://' + hostname + ':81/');
        
        ws.onopen = () => {
            console.log('WebSocket Connected');
            document.getElementById('modeToggle').disabled = false;
        };
        
        ws.onclose = () => {
            console.log('WebSocket Disconnected. Reconnecting...');
            document.getElementById('modeToggle').disabled = true;
            setTimeout(connectWebSocket, 5000);
        };
        
        ws.onerror = (e) => {
            console.error('WebSocket Error', e);
        };
        
        ws.onmessage = (event) => {
            handleWebSocketData(event.data);
        };
    }

    function sendCmd(cmd) {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(cmd);
            console.log("Sent: " + cmd);
        }
    }

    // --- UI/Mode Logic ---

    function toggleMode(checkbox) {
        autoScanActive = checkbox.checked;

        document.getElementById('modeText').textContent = autoScanActive ? 'AUTO SWEEP' : 'MANUAL CONTROL';
        document.getElementById('manualSliders').classList.toggle('disabled-input', autoScanActive);
        document.getElementById('autoDisplay').classList.toggle('disabled-input', !autoScanActive);
        
        const cmd = autoScanActive ? 'MODE:AUTO' : 'MODE:MANUAL';
        sendCmd(cmd);
    }

    function updateServo(type, value) {
        if (!autoScanActive) {
            if (type === 'PAN') {
                document.getElementById('panValue').textContent = value;
                sendCmd('PAN:' + value);
            } else if (type === 'TILT') {
                document.getElementById('tiltValue').textContent = value;
                sendCmd('TILT:' + value);
            }
        }
    }

    // --- Data Handling and Radar Drawing ---

    function handleWebSocketData(data) {
        if (!data.startsWith('DATA:')) return;

        // DATA:PAN_ANGLE|DISTANCE_CM|TILT_ANGLE
        const parts = data.substring(5).split('|');
        if (parts.length < 3) return;

        const panAngle = parseFloat(parts[0]);
        const distanceCm = parseFloat(parts[1]);
        const tiltAngle = parseFloat(parts[2]);

        // Update display values based on mode
        if (autoScanActive) {
            document.getElementById('autoPanValue').textContent = panAngle.toFixed(0);
            document.getElementById('autoTiltValue').textContent = tiltAngle.toFixed(0);
        } else {
            // Update Manual UI to reflect actual robot position
            document.getElementById('panValue').textContent = panAngle.toFixed(0);
            document.getElementById('panSlider').value = panAngle.toFixed(0); 
            document.getElementById('tiltValue').textContent = tiltAngle.toFixed(0);
            document.getElementById('tiltSlider').value = tiltAngle.toFixed(0); 
        }

        // Update Distance Display
        document.getElementById('currentDistance').textContent = distanceCm.toFixed(1);

        // Draw Radar
        drawRadar(panAngle, distanceCm);
    }

    function drawRadar(panAngle, distanceCm) {
        // Clear canvas
        CTX.clearRect(0, 0, RADAR_CANVAS.width, RADAR_CANVAS.height);

        // 1. Draw Radar Arc Background
        CTX.beginPath();
        CTX.arc(CENTER_X, CENTER_Y, MAX_RADIUS, Math.PI, 2 * Math.PI);
        CTX.fillStyle = '#1f2937'; 
        CTX.fill();
        CTX.closePath();

        // 2. Draw Grid Lines
        CTX.strokeStyle = '#4b5563'; 
        CTX.lineWidth = 1;

        // Range Rings (25cm, 50cm, 75cm, 100cm)
        for (let i = 1; i <= 4; i++) {
            const radius = (i / 4.0) * MAX_RADIUS;
            CTX.beginPath();
            CTX.arc(CENTER_X, CENTER_Y, radius, Math.PI, 2 * Math.PI);
            CTX.stroke();
            CTX.closePath();
            
            // Draw range label
            CTX.fillStyle = '#6b7280';
            CTX.font = '10px Arial';
            CTX.fillText((i * 25) + 'cm', CENTER_X + radius * 0.98 * Math.cos(Math.PI * 1.05), CENTER_Y + radius * 0.98 * Math.sin(Math.PI * 1.05));
        }

        // Angle Lines (0, 45, 90, 135, 180)
        for (let i = 0; i <= 180; i += 45) {
            const rad = i * Math.PI / 180;
            CTX.beginPath();
            CTX.moveTo(CENTER_X, CENTER_Y);
            CTX.lineTo(CENTER_X + MAX_RADIUS * Math.cos(rad + Math.PI), CENTER_Y + MAX_RADIUS * Math.sin(rad + Math.PI));
            CTX.stroke();
            CTX.closePath();
        }

        // 3. Draw Scan Line (Green)
        const scanRad = panAngle * Math.PI / 180;
        CTX.beginPath();
        CTX.moveTo(CENTER_X, CENTER_Y);
        CTX.lineTo(CENTER_X + MAX_RADIUS * Math.cos(scanRad + Math.PI), CENTER_Y + MAX_RADIUS * Math.sin(scanRad + Math.PI));
        CTX.strokeStyle = '#10b981'; // Green
        CTX.lineWidth = 3; // Thicker scan line
        CTX.stroke();
        CTX.closePath();

        // 4. Draw Target Dot (Red)
        if (distanceCm > 0 && distanceCm <= RADAR_RANGE_CM) {
            // Map distance (0 to 100cm) to radius (0 to MAX_RADIUS)
            const targetRadius = (distanceCm / RADAR_RANGE_CM) * MAX_RADIUS;
            
            CTX.beginPath();
            const targetX = CENTER_X + targetRadius * Math.cos(scanRad + Math.PI);
            const targetY = CENTER_Y + targetRadius * Math.sin(scanRad + Math.PI);

            CTX.arc(targetX, targetY, 6, 0, 2 * Math.PI); // Larger dot
            CTX.fillStyle = '#ef4444'; // Red
            CTX.fill();
            CTX.closePath();
        }
        
        // 5. Draw Center Pin
        CTX.beginPath();
        CTX.arc(CENTER_X, CENTER_Y, 4, 0, 2 * Math.PI);
        CTX.fillStyle = '#6366f1'; // Indigo
        CTX.fill();
        CTX.closePath();
    }


    // --- Initialization ---
    window.onload = () => {
        connectWebSocket();
        toggleMode(document.getElementById('modeToggle')); // Initialize UI state
        // Set initial manual slider values
        document.getElementById('panSlider').value = 0;
        // Setting manual tilt slider to 90 for the center of the new 0-180 range
        document.getElementById('tiltSlider').value = 90; 
        RADAR_CANVAS.style.width = '100%';
        RADAR_CANVAS.style.height = 'auto'; 
    };

</script>
</body>
</html>
)rawliteral";

// ====================================================================
//                             DRIVING LOGIC
// ====================================================================

void setupMotorPins() {
    pinMode(MOTOR_EN_PIN, OUTPUT);
    pinMode(R_FORWARD_PIN, OUTPUT);
    pinMode(R_BACKWARD_PIN, OUTPUT);
    pinMode(L_FORWARD_PIN, OUTPUT);
    pinMode(L_BACKWARD_PIN, OUTPUT);
    digitalWrite(MOTOR_EN_PIN, LOW);
}

void setMotor(int rFwd, int rBwd, int lFwd, int lBwd) {
    digitalWrite(R_FORWARD_PIN, rFwd);
    digitalWrite(R_BACKWARD_PIN, rBwd);
    digitalWrite(L_FORWARD_PIN, lFwd);
    digitalWrite(L_BACKWARD_PIN, lBwd);
    
    // Enable/Disable the motor driver
    digitalWrite(MOTOR_EN_PIN, rFwd || rBwd || lFwd || lBwd ? HIGH : LOW);
}

void moveForward()    { setMotor(HIGH, LOW,  HIGH, LOW);   Serial.println("DRIVE: Forward"); }
void moveBackward()   { setMotor(LOW,  HIGH, LOW,  HIGH);  Serial.println("DRIVE: Backward"); }
void turnLeft()       { setMotor(HIGH, LOW,  LOW,  HIGH);  Serial.println("DRIVE: Turn Left"); }
void turnRight()      { setMotor(LOW,  HIGH, HIGH, LOW);   Serial.println("DRIVE: Turn Right"); }
void stopDrive()      { setMotor(LOW,  LOW,  LOW,  LOW);   Serial.println("DRIVE: Stop"); }


// ====================================================================
//                          SENSOR & LASER LOGIC
// ====================================================================

long measureDistance() {
  // Use a short pulse duration to avoid issues
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Use a timeout of 30ms (max range ~5m)
  long duration = pulseIn(echoPin, HIGH, 30000); 
  long distanceCm = duration * 0.0343 / 2;
  return distanceCm;
}

// Function for auto-mode fire (a single pulse)
void singleShot() {
  Serial.println("LASER: AUTO FIRE! (Pulse)");
  digitalWrite(laserPin, HIGH); // Laser ON
  delay(SINGLE_SHOT_TIME);
  digitalWrite(laserPin, LOW);  // Laser OFF
}

// Handler for the machine gun effect (non-blocking)
void handleContinuousFire() {
    if (isManualFiring) {
        // Check if enough time has passed to toggle the laser state
        if (millis() - lastLaserToggleTime >= laserToggleInterval) {
            // Toggle the laser pin state (ON -> OFF, or OFF -> ON)
            int currentState = digitalRead(laserPin);
            digitalWrite(laserPin, !currentState); 
            lastLaserToggleTime = millis();
        }
    }
}

// Functions for manual, continuous fire (start/stop)
void startManualFire() {
    isManualFiring = true;
    lastLaserToggleTime = millis(); // Reset timer to start pulsing immediately
    // Initial state: turn it ON immediately
    digitalWrite(laserPin, HIGH);
    Serial.println("LASER: MANUAL FIRE START (Pulsing)");
}

void stopManualFire() {
    isManualFiring = false;
    digitalWrite(laserPin, LOW); // Ensure laser is OFF when stopped
    Serial.println("LASER: MANUAL FIRE STOP");
}


// ====================================================================
//                           SCANNING LOGIC (FULL SWEEP REINSTATED)
// ====================================================================

void runScanStep() {
    // 1. Move to the current physical position
    // FIX: Invert the angle sent to the physical servo (180-angle) to align
    // 0 on UI (left) with 0 on physical axis, assuming servo is mounted 180 degrees off.
    int physicalPanAngle = panEnd - currentPanAngle; 
    
    servoPan.write(physicalPanAngle); 
    servoTilt.write(currentTiltAngle);
    delay(servoSettleTime);

    // 2. Measurement and Action
    long distance = measureDistance();
    
    // Auto shooting (within 100cm)
    if (distance < SHOOT_DISTANCE && distance > 0) { 
      singleShot(); 
    } 

    // 3. Send Radar Data to Web Client
    String data = "DATA:";
    data += String(currentPanAngle) + "|" + String(distance) + "|" + String(currentTiltAngle);
    webSocket.broadcastTXT(data);

    // 4. Update position for the *next* step (The full sweep logic)

    if (panDirection) {
      // Sweeping 0 -> 180 (Logically Left to Right)
      currentPanAngle += panStep;
      if (currentPanAngle >= panEnd) {
        panDirection = false; // Reverse direction
        currentPanAngle = panEnd; 
      }
    } else {
      // Sweeping 180 -> 0 (Logically Right to Left)
      currentPanAngle -= panStep;
      if (currentPanAngle <= panStart) {
        panDirection = true; // Reverse direction
        currentPanAngle = panStart; 
        
        // --- TILT MOVEMENT LOGIC (Only moves after a full 180-0 pan sweep) ---
        // Now moves in steps up (70->120) and then in steps down (120->70)
        if (tiltDirection) {
          currentTiltAngle += tiltStep;
          if (currentTiltAngle >= tiltEnd) {
            currentTiltAngle = tiltEnd; // Cap at max
            tiltDirection = false;     // Now move down
          }
        } else {
          currentTiltAngle -= tiltStep;
          if (currentTiltAngle <= tiltStart) {
            currentTiltAngle = tiltStart; // Cap at min
            tiltDirection = true;      // Now move up
          }
        }
      }
    }
    
    // Old step 5 is removed as the new logic handles tilt boundaries internally.
}


// ====================================================================
//                            WEB SOCKET HANDLER
// ====================================================================

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    if(type != WStype_TEXT) return;

    String cmd = "";
    for(size_t i = 0; i < length; i++) cmd += (char)payload[i];
    cmd.trim();

    Serial.print("WS Command: ");
    Serial.println(cmd);

    // --- Driving Commands (Hold-to-Move) ---
    if(cmd == "F_START")       moveForward();
    else if(cmd == "B_START")  moveBackward();
    else if(cmd == "L_START")  turnLeft();
    else if(cmd == "R_START")  turnRight();
    else if(cmd == "S_STOP")   stopDrive();
    
    // --- Laser Fire (Machine Gun) ---
    else if(cmd == "FIRE_START") startManualFire();
    else if(cmd == "FIRE_STOP")  stopManualFire();

    // --- Mode Control ---
    else if(cmd == "MODE:MANUAL") {
      autoScanMode = false;
      // Set physical servos to current stored logical angle
      servoPan.write(panEnd - currentPanAngle);
      servoTilt.write(currentTiltAngle);
    }
    else if(cmd == "MODE:AUTO") {
      autoScanMode = true;
      // Reset scan position to start when entering AUTO
      currentPanAngle = panStart;
      currentTiltAngle = tiltStart;
      panDirection = true;
      tiltDirection = true; // Reset tilt direction to start going UP
      servoPan.write(panEnd - currentPanAngle); 
      servoTilt.write(currentTiltAngle);
    }

    // --- Manual Servo Control (if in Manual Mode) ---
    else if (cmd.startsWith("PAN:")) {
      int angle = cmd.substring(4).toInt();
      if (!autoScanMode) {
        angle = constrain(angle, panStart, panEnd);
        int invertedAngle = panEnd - angle; 
        servoPan.write(invertedAngle);
        currentPanAngle = angle; // Store non-inverted angle for UI/data reporting
      }
    }
    else if (cmd.startsWith("TILT:")) {
      int angle = cmd.substring(5).toInt();
      if (!autoScanMode) {
        // UPDATED: Allow full 0-180 manual control
        angle = constrain(angle, 0, 180); 
        servoTilt.write(angle);
        currentTiltAngle = angle; // Store for UI/data reporting
      }
    }
}


// ====================================================================
//                                SETUP
// ====================================================================

void setup() {
    Serial.begin(115200);

    // Setup Driving Pins
    setupMotorPins();

    // Setup Sensor & Laser Pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(laserPin, OUTPUT);
    digitalWrite(laserPin, LOW); // Start with laser OFF

    // Initialize Servos
    // Allocate two separate timers for the two servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    // Standard servo min/max pulse widths for full 0-180 degree travel (500us to 2500us)
    servoPan.attach(panServoPin, 500, 2500); 
    servoTilt.attach(tiltServoPin, 500, 2500);

    // Set initial servo positions
    servoPan.write(panEnd - panStart); // Write inverted 0 degrees
    servoTilt.write(tiltStart); 
    delay(1000); 

    // --- WiFi Setup ---
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    int attempts = 0;
    while(WiFi.status() != WL_CONNECTED && attempts++ < 20) {
        delay(500);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi connected. IP:");
      Serial.println(WiFi.localIP());
      
      // NEW: Start mDNS service, allowing access via http://esp32robot.local
      if (MDNS.begin(host)) {
        Serial.print("MDNS responder started. Access at: http://");
        Serial.print(host);
        Serial.println(".local");
      } else {
        Serial.println("Error setting up MDNS.");
      }
      
    } else {
      Serial.println("\nWiFi connection failed. Check credentials.");
    }

    // --- Web Server Setup ---
    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", HTML_CONTENT); 
    });

    webSocket.begin();
    webSocket.onEvent(onWebSocketEvent);
    server.begin();
}


// ====================================================================
//                                LOOP
// ====================================================================

void loop() {
    server.handleClient();
    webSocket.loop();
    
    // Handle the non-blocking machine gun fire effect
    handleContinuousFire();

    if (autoScanMode) {
        runScanStep();
    } else {
        // In Manual Mode, we still need to send data to the radar display
        long distance = measureDistance();
        // Report current manual angles and distance back to UI
        String data = "DATA:";
        data += String(currentPanAngle) + "|" + String(distance) + "|" + String(currentTiltAngle);
        webSocket.broadcastTXT(data);
        delay(100); 
    }
}
