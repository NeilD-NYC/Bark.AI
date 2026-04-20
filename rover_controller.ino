// ============================================================
// ESP32 Autonomous Rover Controller v2.0
// 2WD Property Patrol System - Rear-Wheel Drive
// BLDC Hub Motor ESC Control via 50Hz Servo PWM
// Interfaces with Raspberry Pi 5 + Hailo NPU via Serial2
// No encoder/PID - closed-loop handled internally by ESCs
//
// HARDWARE ASSUMED:
//   - 2x BLDC hub motors (hoverboard or ebike-derived) with
//     brushless ESCs accepting standard RC servo PWM signal
//   - Raspberry Pi 5 connected via UART (Serial2)
//   - Hailo NPU on RPi5 - sends detection events over serial
//   - 36V Li-ion pack with voltage divider on ADC pin
//   - Crazy Cart chassis, rear-wheel drive only
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// ============================================================
// WiFi CREDENTIALS
// ============================================================
const char* WIFI_SSID     = "YOUR_SSID";
const char* WIFI_PASSWORD = "YOUR_PASSWORD";

// ============================================================
// PIN DEFINITIONS  (all conflicts resolved)
// ============================================================

// ESC PWM signal outputs (50Hz servo-style)
#define ESC_LEFT_PIN        25   // Rear-Left ESC signal wire
#define ESC_RIGHT_PIN       26   // Rear-Right ESC signal wire

// Raspberry Pi 5 UART (Serial2 remapped - no USB conflict)
#define RPI_TX_PIN          17   // ESP32 TX -> RPi5 RX
#define RPI_RX_PIN          16   // ESP32 RX -> RPi5 TX

// Handshake / interrupt lines from RPi5 and Hailo
#define RPI_READY_PIN        4   // RPi signals it has data ready
#define NPU_ALERT_PIN        5   // Hailo NPU signals detection event

// Battery voltage sense (ADC1 only - ADC2 conflicts with WiFi)
// Wire 36V pack through resistor divider: 100k (top) + 10k (bottom)
// Vout = Vin * 10k/110k = Vin * 0.0909  =>  36V -> 3.27V (safe)
#define BATTERY_ADC_PIN     34   // GPIO34: input-only, ADC1_CH6

// Onboard status LED
#define STATUS_LED_PIN       2   // Built-in LED on most ESP32 dev boards

// ============================================================
// LEDC (PWM) CONFIGURATION FOR ESCs
// ============================================================
#define ESC_LEFT_CH          0   // LEDC channel 0
#define ESC_RIGHT_CH         1   // LEDC channel 1
#define ESC_FREQ_HZ         50   // Standard RC ESC signal frequency
#define ESC_RESOLUTION      16   // 16-bit: 0-65535 ticks per 20ms period

// ESC pulse widths in microseconds (standard RC/BLDC range)
// Tune NEUTRAL if motors creep at rest (ESC calibration)
#define ESC_FULL_FWD_US   2000
#define ESC_NEUTRAL_US    1500   // Stop / armed idle
#define ESC_FULL_REV_US   1000
#define ESC_DEADBAND_US     30   // +/- us around neutral treated as stop

// ============================================================
// BATTERY CALIBRATION
// ============================================================
#define BATT_DIVIDER_RATIO  0.0909f   // 10k / (100k + 10k)
#define BATT_ADC_VREF       3.3f
#define BATT_ADC_MAX        4095.0f
#define BATT_FULL_V         42.0f     // 36V pack fully charged (10S Li-ion)
#define BATT_EMPTY_V        30.0f     // Safe minimum (10S Li-ion cutoff)
#define BATT_LOW_PCT        25.0f
#define BATT_CRIT_PCT       10.0f

// ============================================================
// ROVER STATE MACHINE
// ============================================================
enum RoverState {
    STATE_IDLE,
    STATE_ARMING,        // ESC arming sequence on boot
    STATE_PATROL,
    STATE_INVESTIGATE,
    STATE_ALERT,
    STATE_DETER,
    STATE_RETURN_HOME,
    STATE_LOW_BATTERY,
    STATE_OBSTACLE_AVOID,
    STATE_EMERGENCY_STOP
};

// String labels for telemetry JSON
const char* stateLabel[] = {
    "IDLE", "ARMING", "PATROL", "INVESTIGATE",
    "ALERT", "DETER", "RETURN_HOME", "LOW_BATTERY",
    "OBSTACLE_AVOID", "EMERGENCY_STOP"
};

// ============================================================
// MOTION COMMANDS (received from RPi via serial JSON)
// ============================================================
enum MotionCmd {
    CMD_STOP,
    CMD_FORWARD,
    CMD_BACKWARD,
    CMD_TURN_LEFT,
    CMD_TURN_RIGHT,
    CMD_SPIN_LEFT,
    CMD_SPIN_RIGHT,
    CMD_CUSTOM   // leftSpeed + rightSpeed provided explicitly
};

// ============================================================
// ROVER CONFIGURATION  (tunable at runtime via RPi)
// ============================================================
struct RoverConfig {
    float patrol_speed          = 0.45f;  // 0.0-1.0 fraction of full ESC range
    float max_speed             = 0.85f;  // Cap to protect chassis
    float turn_speed            = 0.35f;
    float investigate_speed     = 0.30f;
    unsigned long patrol_seg_ms = 5000;   // ms between waypoint decisions
    unsigned long deter_dur_ms  = 8000;   // how long to run deter behavior
    unsigned long investigate_ms= 10000;  // how long to investigate before alert
    bool  audio_deter_enabled   = true;   // future: piezo / speaker
};

// ============================================================
// ROVER TELEMETRY
// ============================================================
struct RoverTelemetry {
    RoverState   state               = STATE_IDLE;
    MotionCmd    current_cmd         = CMD_STOP;
    float        battery_percent     = 100.0f;
    float        battery_voltage     = 42.0f;
    int          esc_left_us         = ESC_NEUTRAL_US;
    int          esc_right_us        = ESC_NEUTRAL_US;
    bool         obstacle_detected   = false;
    bool         intruder_detected   = false;
    String       detection_class     = "";
    float        detection_confidence= 0.0f;
    unsigned long last_detection_ms  = 0;
    int          alert_level         = 0;   // 0=none 1=low 2=med 3=high
    unsigned long uptime_ms          = 0;
    bool         rpi_connected       = false;
    unsigned long last_rpi_msg_ms    = 0;
};

// ============================================================
// GLOBALS
// ============================================================
RoverConfig    cfg;
RoverTelemetry telem;
WebServer      httpServer(80);

// Timing
unsigned long lastBatteryRead      = 0;
unsigned long lastTelemetrySend    = 0;
unsigned long lastStateChange      = 0;
unsigned long patrolTimer          = 0;
unsigned long deterTimer           = 0;
unsigned long investigateTimer     = 0;
unsigned long armingStartMs        = 0;

// RPi serial buffer
String  rpiBuffer = "";
bool    rpiDataReady = false;

// ============================================================
// ESC UTILITY: microseconds -> LEDC duty count
// At 50Hz, period = 20,000 us. 16-bit = 65535 ticks.
// duty = (us / 20000.0) * 65535
// ============================================================
inline uint32_t usToDuty(int us) {
    us = constrain(us, ESC_FULL_REV_US, ESC_FULL_FWD_US);
    return (uint32_t)((us / 20000.0f) * 65535.0f);
}

// ============================================================
// LOW-LEVEL ESC CONTROL
// ============================================================
void setESCLeft(int us) {
    us = constrain(us, ESC_FULL_REV_US, ESC_FULL_FWD_US);
    // Apply deadband around neutral
    if (abs(us - ESC_NEUTRAL_US) < ESC_DEADBAND_US) us = ESC_NEUTRAL_US;
    telem.esc_left_us = us;
    ledcWrite(ESC_LEFT_CH, usToDuty(us));
}

void setESCRight(int us) {
    us = constrain(us, ESC_FULL_REV_US, ESC_FULL_FWD_US);
    if (abs(us - ESC_NEUTRAL_US) < ESC_DEADBAND_US) us = ESC_NEUTRAL_US;
    telem.esc_right_us = us;
    ledcWrite(ESC_RIGHT_CH, usToDuty(us));
}

// ============================================================
// CORE DRIVE FUNCTION
// speedFactor: -1.0 (full reverse) to +1.0 (full forward)
// Applied per side for differential steering
// ============================================================
void driveRover(float leftFactor, float rightFactor) {
    // Apply max_speed cap
    leftFactor  = constrain(leftFactor,  -cfg.max_speed, cfg.max_speed);
    rightFactor = constrain(rightFactor, -cfg.max_speed, cfg.max_speed);

    // Map -1.0 to +1.0 -> ESC_FULL_REV_US to ESC_FULL_FWD_US
    // Center = ESC_NEUTRAL_US (1500us)
    // Range each side = 500us
    int leftUS  = ESC_NEUTRAL_US + (int)(leftFactor  * 500.0f);
    int rightUS = ESC_NEUTRAL_US + (int)(rightFactor * 500.0f);

    setESCLeft(leftUS);
    setESCRight(rightUS);
}

// ============================================================
// HIGH-LEVEL MOTION COMMANDS
// ============================================================
void cmdStop() {
    setESCLeft(ESC_NEUTRAL_US);
    setESCRight(ESC_NEUTRAL_US);
    telem.current_cmd = CMD_STOP;
}

void cmdForward(float speed = -1.0f) {
    float spd = (speed < 0) ? cfg.patrol_speed : constrain(speed, 0.0f, cfg.max_speed);
    driveRover(spd, spd);
    telem.current_cmd = CMD_FORWARD;
}

void cmdBackward(float speed = -1.0f) {
    float spd = (speed < 0) ? cfg.patrol_speed : constrain(speed, 0.0f, cfg.max_speed);
    driveRover(-spd, -spd);
    telem.current_cmd = CMD_BACKWARD;
}

void cmdTurnLeft(float speed = -1.0f) {
    // Gradual arc: right side faster
    float spd = (speed < 0) ? cfg.turn_speed : constrain(speed, 0.0f, cfg.max_speed);
    driveRover(spd * 0.4f, spd);
    telem.current_cmd = CMD_TURN_LEFT;
}

void cmdTurnRight(float speed = -1.0f) {
    float spd = (speed < 0) ? cfg.turn_speed : constrain(speed, 0.0f, cfg.max_speed);
    driveRover(spd, spd * 0.4f);
    telem.current_cmd = CMD_TURN_RIGHT;
}

void cmdSpinLeft(float speed = -1.0f) {
    // Counter-rotate: left back, right forward
    float spd = (speed < 0) ? cfg.turn_speed : constrain(speed, 0.0f, cfg.max_speed);
    driveRover(-spd, spd);
    telem.current_cmd = CMD_SPIN_LEFT;
}

void cmdSpinRight(float speed = -1.0f) {
    float spd = (speed < 0) ? cfg.turn_speed : constrain(speed, 0.0f, cfg.max_speed);
    driveRover(spd, -spd);
    telem.current_cmd = CMD_SPIN_RIGHT;
}

// Custom per-side control (used by RPi for precise navigation)
void cmdCustom(float leftFactor, float rightFactor) {
    driveRover(leftFactor, rightFactor);
    telem.current_cmd = CMD_CUSTOM;
}

// ============================================================
// ESC ARMING SEQUENCE
// Most brushless ESCs require neutral signal for ~2s before
// accepting throttle commands. Call once at startup.
// ============================================================
void armESCs() {
    Serial.println("[ESC] Arming sequence start - sending neutral for 2500ms");
    setESCLeft(ESC_NEUTRAL_US);
    setESCRight(ESC_NEUTRAL_US);
    // Non-blocking arming: caller checks elapsed time
    armingStartMs = millis();
    telem.state = STATE_ARMING;
}

bool isArmed() {
    return (millis() - armingStartMs) >= 2500;
}

// ============================================================
// BATTERY MONITORING
// ADC1 only (ADC2 disabled when WiFi active on ESP32)
// Call frequently; internal smoothing applied
// ============================================================
void updateBattery() {
    if (millis() - lastBatteryRead < 5000) return;  // Read every 5s
    lastBatteryRead = millis();

    // Oversample ADC to reduce ESP32 ADC noise (known hardware issue)
    long adcSum = 0;
    const int SAMPLES = 16;
    for (int i = 0; i < SAMPLES; i++) {
        adcSum += analogRead(BATTERY_ADC_PIN);
        delayMicroseconds(50);
    }
    float adcAvg = adcSum / (float)SAMPLES;

    // Convert to actual pack voltage
    float adcVoltage = (adcAvg / BATT_ADC_MAX) * BATT_ADC_VREF;
    float packVoltage = adcVoltage / BATT_DIVIDER_RATIO;

    // Smooth with previous reading (IIR)
    telem.battery_voltage = telem.battery_voltage * 0.7f + packVoltage * 0.3f;

    // Map voltage to percentage (linear approximation)
    telem.battery_percent = constrain(
        ((telem.battery_voltage - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V)) * 100.0f,
        0.0f, 100.0f
    );

    // Trigger low battery state
    if (telem.battery_percent <= BATT_CRIT_PCT) {
        telem.state = STATE_EMERGENCY_STOP;
        cmdStop();
        Serial.println("[BATT] CRITICAL - emergency stop");
    } else if (telem.battery_percent <= BATT_LOW_PCT && telem.state != STATE_LOW_BATTERY) {
        telem.state = STATE_LOW_BATTERY;
        Serial.println("[BATT] LOW - returning home");
    }
}

// ============================================================
// STATE MACHINE
// ============================================================
void changeState(RoverState newState) {
    if (newState == telem.state) return;
    Serial.printf("[STATE] %s -> %s\n", stateLabel[telem.state], stateLabel[newState]);
    telem.state = newState;
    lastStateChange = millis();
}

void runStateMachine() {
    unsigned long now = millis();
    unsigned long timeInState = now - lastStateChange;

    switch (telem.state) {

        case STATE_ARMING:
            if (isArmed()) {
                Serial.println("[ESC] Armed. Transitioning to IDLE.");
                changeState(STATE_IDLE);
            }
            break;

        case STATE_IDLE:
            cmdStop();
            // RPi will command PATROL when ready
            break;

        case STATE_PATROL:
            // Basic time-sliced patrol: move forward, then check for navigation
            // RPi/Hailo drives actual path decisions; ESP32 executes them
            if (now - patrolTimer >= cfg.patrol_seg_ms) {
                patrolTimer = now;
                // Heartbeat to RPi requesting next waypoint command
                Serial2.println("{\"req\":\"next_waypoint\"}");
            }
            if (telem.intruder_detected) {
                changeState(STATE_INVESTIGATE);
                investigateTimer = now;
            }
            if (telem.obstacle_detected) {
                changeState(STATE_OBSTACLE_AVOID);
            }
            break;

        case STATE_INVESTIGATE:
            // Slow approach toward detection zone
            cmdForward(cfg.investigate_speed);
            if (timeInState >= cfg.investigate_ms) {
                if (telem.alert_level >= 2) {
                    changeState(STATE_ALERT);
                } else {
                    changeState(STATE_PATROL);
                    telem.intruder_detected = false;
                }
            }
            break;

        case STATE_ALERT:
            cmdStop();
            // RPi handles notification (camera, push alert)
            // ESP32 triggers deterrent after brief pause
            if (timeInState >= 1500 && cfg.audio_deter_enabled) {
                changeState(STATE_DETER);
                deterTimer = now;
            }
            break;

        case STATE_DETER:
            // Spin in place to appear active / intimidating
            if ((now / 600) % 2 == 0) {
                cmdSpinLeft(0.30f);
            } else {
                cmdSpinRight(0.30f);
            }
            if (timeInState >= cfg.deter_dur_ms) {
                cmdStop();
                telem.intruder_detected = false;
                telem.alert_level = 0;
                changeState(STATE_PATROL);
            }
            break;

        case STATE_OBSTACLE_AVOID:
            // Simple 3-phase avoidance: back up, spin, resume
            if (timeInState < 1200) {
                cmdBackward(0.35f);
            } else if (timeInState < 2200) {
                cmdSpinRight(0.35f);
            } else {
                telem.obstacle_detected = false;
                changeState(STATE_PATROL);
            }
            break;

        case STATE_RETURN_HOME:
            // RPi navigates home via waypoints; ESP32 just executes cmds
            // If no RPi command arrives for 3s, stop and wait
            if ((now - telem.last_rpi_msg_ms) > 3000) {
                cmdStop();
            }
            break;

        case STATE_LOW_BATTERY:
            cmdStop();
            // Signal RPi to initiate return-home navigation
            Serial2.println("{\"req\":\"return_home\",\"reason\":\"low_battery\"}");
            changeState(STATE_RETURN_HOME);
            break;

        case STATE_EMERGENCY_STOP:
            cmdStop();
            digitalWrite(STATUS_LED_PIN, HIGH);  // Solid LED = critical fault
            // Do not leave this state without RPi reset command
            break;
    }
}

// ============================================================
// RPi SERIAL COMMUNICATION
// JSON protocol:
//   RPi -> ESP32: {"cmd":"FORWARD","speed":0.5}
//                 {"cmd":"CUSTOM","left":0.4,"right":-0.2}
//                 {"cmd":"STATE","state":"PATROL"}
//                 {"detection":{"class":"person","conf":0.93,"alert":2}}
//                 {"obstacle":true}
//                 {"config":{"patrol_speed":0.5,"max_speed":0.8}}
//   ESP32 -> RPi: {"state":"PATROL","batt_pct":82.1,"batt_v":36.4,
//                  "esc_l":1700,"esc_r":1700,"alert":0,"uptime":12345}
// ============================================================
void parseRPiMessage(const String& msg) {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, msg);
    if (err) {
        Serial.printf("[RPI] JSON parse error: %s | msg: %s\n", err.c_str(), msg.c_str());
        return;
    }

    telem.last_rpi_msg_ms = millis();
    telem.rpi_connected   = true;

    // Motion command
    if (doc.containsKey("cmd")) {
        const char* cmd = doc["cmd"];
        float speed = doc["speed"] | -1.0f;

        if      (strcmp(cmd, "STOP")      == 0) { cmdStop();              changeState(STATE_IDLE); }
        else if (strcmp(cmd, "FORWARD")   == 0) { cmdForward(speed);      changeState(STATE_PATROL); }
        else if (strcmp(cmd, "BACKWARD")  == 0) { cmdBackward(speed); }
        else if (strcmp(cmd, "TURN_LEFT") == 0) { cmdTurnLeft(speed); }
        else if (strcmp(cmd, "TURN_RIGHT")== 0) { cmdTurnRight(speed); }
        else if (strcmp(cmd, "SPIN_LEFT") == 0) { cmdSpinLeft(speed); }
        else if (strcmp(cmd, "SPIN_RIGHT")== 0) { cmdSpinRight(speed); }
        else if (strcmp(cmd, "CUSTOM")    == 0) {
            float l = doc["left"]  | 0.0f;
            float r = doc["right"] | 0.0f;
            cmdCustom(l, r);
        }
        else if (strcmp(cmd, "STATE") == 0) {
            const char* st = doc["state"];
            if      (strcmp(st, "PATROL")      == 0) changeState(STATE_PATROL);
            else if (strcmp(st, "IDLE")        == 0) changeState(STATE_IDLE);
            else if (strcmp(st, "RETURN_HOME") == 0) changeState(STATE_RETURN_HOME);
            else if (strcmp(st, "E_STOP")      == 0) { cmdStop(); changeState(STATE_EMERGENCY_STOP); }
        }
    }

    // Detection event from Hailo NPU (relayed by RPi)
    if (doc.containsKey("detection")) {
        JsonObject det = doc["detection"];
        telem.detection_class      = det["class"].as<String>();
        telem.detection_confidence = det["conf"] | 0.0f;
        telem.alert_level          = det["alert"] | 0;
        telem.intruder_detected    = (telem.alert_level > 0);
        telem.last_detection_ms    = millis();
        Serial.printf("[DETECT] %s %.2f alert=%d\n",
            telem.detection_class.c_str(), telem.detection_confidence, telem.alert_level);
    }

    // Obstacle flag from RPi depth/ultrasonic sensor
    if (doc.containsKey("obstacle")) {
        telem.obstacle_detected = doc["obstacle"].as<bool>();
    }

    // Runtime config update
    if (doc.containsKey("config")) {
        JsonObject c = doc["config"];
        if (c.containsKey("patrol_speed")) cfg.patrol_speed   = c["patrol_speed"];
        if (c.containsKey("max_speed"))    cfg.max_speed       = c["max_speed"];
        if (c.containsKey("turn_speed"))   cfg.turn_speed      = c["turn_speed"];
        if (c.containsKey("deter_ms"))     cfg.deter_dur_ms    = c["deter_ms"];
        Serial.println("[CFG] Config updated");
    }
}

void readRPiSerial() {
    while (Serial2.available()) {
        char c = Serial2.read();
        if (c == '\n') {
            rpiBuffer.trim();
            if (rpiBuffer.length() > 0) {
                parseRPiMessage(rpiBuffer);
            }
            rpiBuffer = "";
        } else {
            if (rpiBuffer.length() < 512) rpiBuffer += c;
        }
    }

    // RPi connection watchdog: if no message in 10s, flag disconnected
    if (millis() - telem.last_rpi_msg_ms > 10000) {
        telem.rpi_connected = false;
    }
}

void sendTelemetryToRPi() {
    if (millis() - lastTelemetrySend < 500) return;  // Send at 2Hz
    lastTelemetrySend = millis();

    StaticJsonDocument<256> doc;
    doc["state"]    = stateLabel[telem.state];
    doc["batt_pct"] = (int)telem.battery_percent;
    doc["batt_v"]   = serialized(String(telem.battery_voltage, 1));
    doc["esc_l"]    = telem.esc_left_us;
    doc["esc_r"]    = telem.esc_right_us;
    doc["alert"]    = telem.alert_level;
    doc["obstacle"] = telem.obstacle_detected;
    doc["uptime"]   = millis() / 1000;

    String out;
    serializeJson(doc, out);
    Serial2.println(out);
}

// ============================================================
// HTTP TELEMETRY ENDPOINT (debug / dashboard)
// GET http://<rover-ip>/telemetry
// ============================================================
void handleHttpTelemetry() {
    StaticJsonDocument<512> doc;
    doc["state"]       = stateLabel[telem.state];
    doc["batt_pct"]    = telem.battery_percent;
    doc["batt_v"]      = telem.battery_voltage;
    doc["esc_left_us"] = telem.esc_left_us;
    doc["esc_right_us"]= telem.esc_right_us;
    doc["alert_level"] = telem.alert_level;
    doc["intruder"]    = telem.intruder_detected;
    doc["obstacle"]    = telem.obstacle_detected;
    doc["rpi_online"]  = telem.rpi_connected;
    doc["detect_class"]= telem.detection_class;
    doc["detect_conf"] = telem.detection_confidence;
    doc["uptime_s"]    = millis() / 1000;

    String body;
    serializeJsonPretty(doc, body);
    httpServer.send(200, "application/json", body);
}

void handleHttpStop() {
    cmdStop();
    changeState(STATE_IDLE);
    httpServer.send(200, "application/json", "{\"ok\":true,\"cmd\":\"STOP\"}");
}

void handleHttpNotFound() {
    httpServer.send(404, "text/plain", "Not found");
}

void setupWebServer() {
    httpServer.on("/telemetry", handleHttpTelemetry);
    httpServer.on("/stop",      handleHttpStop);
    httpServer.onNotFound(handleHttpNotFound);
    httpServer.begin();
    Serial.println("[HTTP] Web server started");
}

// ============================================================
// WiFi SETUP
// ============================================================
void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    unsigned long wStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wStart < 10000) {
        delay(500);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[WiFi] Connection failed - running in offline mode");
    }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n[BOOT] ESP32 Rover Controller v2.0");

    // GPIO setup
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(RPI_READY_PIN,  INPUT_PULLDOWN);
    pinMode(NPU_ALERT_PIN,  INPUT_PULLDOWN);
    pinMode(BATTERY_ADC_PIN, INPUT);
    analogReadResolution(12);  // 12-bit ADC (0-4095)

    // Configure ESC LEDC channels at 50Hz, 16-bit resolution
    ledcSetup(ESC_LEFT_CH,  ESC_FREQ_HZ, ESC_RESOLUTION);
    ledcSetup(ESC_RIGHT_CH, ESC_FREQ_HZ, ESC_RESOLUTION);
    ledcAttachPin(ESC_LEFT_PIN,  ESC_LEFT_CH);
    ledcAttachPin(ESC_RIGHT_PIN, ESC_RIGHT_CH);

    // RPi UART on Serial2 with remapped pins
    Serial2.begin(115200, SERIAL_8N1, RPI_RX_PIN, RPI_TX_PIN);
    Serial.printf("[UART] Serial2 -> RPi5 on TX:%d RX:%d\n", RPI_TX_PIN, RPI_RX_PIN);

    // WiFi + web server
    setupWiFi();
    setupWebServer();

    // Initial battery read
    lastBatteryRead = 0;
    updateBattery();
    Serial.printf("[BATT] Initial: %.1fV (%.0f%%)\n",
        telem.battery_voltage, telem.battery_percent);

    // ESC arming (non-blocking; state machine handles completion)
    armESCs();

    // Blink LED to signal boot complete
    for (int i = 0; i < 3; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH); delay(150);
        digitalWrite(STATUS_LED_PIN, LOW);  delay(150);
    }

    Serial.println("[BOOT] Setup complete. Arming ESCs...");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
    // Serial comms with RPi5
    readRPiSerial();
    sendTelemetryToRPi();

    // Battery monitoring
    updateBattery();

    // State machine
    runStateMachine();

    // HTTP server
    httpServer.handleClient();

    // Update uptime
    telem.uptime_ms = millis();

    // Status LED heartbeat (fast blink = emergency, slow = normal)
    unsigned long now = millis();
    if (telem.state == STATE_EMERGENCY_STOP) {
        digitalWrite(STATUS_LED_PIN, (now / 200) % 2);
    } else if (telem.state == STATE_ALERT || telem.state == STATE_DETER) {
        digitalWrite(STATUS_LED_PIN, (now / 400) % 2);
    } else {
        digitalWrite(STATUS_LED_PIN, (now / 1500) % 2);
    }
}
