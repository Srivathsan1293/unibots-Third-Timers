#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Encoder.h>

// =========================
// Pin Definitions
// =========================

// --- Drive Pins ---
const int FL_IN1 = 13, FL_IN2 = 14;
const int FR_IN2 = 12, FR_IN1 = 26; // FR_IN1 moved to 12
const int LR_IN3 = 27, LR_IN4 = 32;
const int RR_IN4 = 18, RR_IN3 = 33;

// --- PWM Pins ---
const int FL_PWM = 2;
const int FR_PWM = 5;
const int LR_PWM = 15;
const int RR_PWM = 22;

// --- Encoder Pins ---
const int FL_A = 34, FL_B = 35;
const int FR_A = 36, FR_B = 39;
const int RR_A = 4,  RR_B = 25; // RR swapped to 4 and 25
const int LR_PIN = 19;

// =========================
// PWM Setup
// =========================
const int PWM_FREQ = 20000;
const int PWM_RES_BITS = 8;

// =========================
// Encoder Objects
// =========================
ESP32Encoder encFL, encFR, encRR;
volatile long pulseCountLR = 0;

// =========================
// IMU Setup (For PID Correction ONLY)
// =========================
Adafruit_MPU6050 mpu;
float currentYaw = 17.0f; 
float gyroZOffset = 0.0f;
unsigned long lastImuTime = 0;

// =========================
// Drive State Machine & Control
// =========================
enum DriveState { STOPPED, FWD, REV, LEFT, RIGHT };
DriveState currentState = STOPPED;
DriveState pendingState = STOPPED;

float activeTargetPPS = 0.0f; 
float currentRampedPPS = 0.0f;
const float maxAccelPerLoop = 3000.0f; // Slew rate

float Kp = 0.5f;
float Ki = 0.05f;
float intFL = 0.0f, intFR = 0.0f, intLR = 0.0f, intRR = 0.0f;

float Kp_yaw = 15.0f;
float Ki_yaw = 0.0f;
float targetYaw = 0.0f;
float integralYaw = 0.0f;
bool isTurning = false;

// =========================
// Odometry (Pure Encoder Kinematics)
// =========================
float posX = 0.0f;
float posY = 0.0f;
float odomYaw = 0.0f; // New purely hardware-calculated yaw

float mmPerPulse = 0.071267f;
const float trackWidthMM = 160.0f; // Distance between left and right wheels (Update if necessary)
bool odomValid = true;

// =========================
// RTOS Sync
// =========================
TaskHandle_t ControlTaskHandle;
portMUX_TYPE rtosMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE isrMux = portMUX_INITIALIZER_UNLOCKED;

// =========================
// Serial input buffer
// =========================
String serialLine = "";

// =========================
// ISR
// =========================
void IRAM_ATTR isrLR() {
  portENTER_CRITICAL_ISR(&isrMux);
  pulseCountLR++;
  portEXIT_CRITICAL_ISR(&isrMux);
}

// =========================
// Function Prototypes
// =========================
void ControlLoopTask(void * pvParameters);
void setDirections(bool fl, bool fr, bool lr, bool rr);
int getPID(float target, float current, float &i);
void writeMotorPWM(int fl, int fr, int lr, int rr);
void stopAllMotors();
float calibrateGyroZOffset();
void handleSerialInput();
void processCommand(String cmd);

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  delay(200);

  // -------------------------
  // Encoder setup
  // -------------------------
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  encFL.attachHalfQuad(FL_A, FL_B);
  encFR.attachHalfQuad(FR_A, FR_B);
  encRR.attachHalfQuad(RR_A, RR_B);

  encFL.clearCount();
  encFR.clearCount();
  encRR.clearCount();

  pinMode(LR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LR_PIN), isrLR, CHANGE);

  // -------------------------
  // Motor GPIO setup
  // -------------------------
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(LR_IN3, OUTPUT); pinMode(LR_IN4, OUTPUT);
  pinMode(RR_IN3, OUTPUT); pinMode(RR_IN4, OUTPUT);

  // -------------------------
  // PWM setup
  // -------------------------
  ledcAttach(FL_PWM, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(FR_PWM, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(LR_PWM, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(RR_PWM, PWM_FREQ, PWM_RES_BITS);

  stopAllMotors();

  // -------------------------
  // IMU setup (Strictly for immediate loop correction)
  // -------------------------
  Wire.begin(21, 23);
  if (!mpu.begin()) {
    while (1) {
      Serial.println("X:INV,Y:INV,H:0.0");
      delay(500);
    }
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  gyroZOffset = calibrateGyroZOffset();

  portENTER_CRITICAL(&rtosMux);
  currentYaw = 0.0f;
  targetYaw = 0.0f;
  integralYaw = 0.0f;
  portEXIT_CRITICAL(&rtosMux);

  lastImuTime = millis();

  // -------------------------
  // Control task on Core 0
  // -------------------------
  xTaskCreatePinnedToCore(
    ControlLoopTask,
    "ControlTask",
    4096,
    NULL,
    1,
    &ControlTaskHandle,
    0
  );
}

// =========================
// Main loop: serial + telemetry
// =========================
void loop() {
  handleSerialInput();

  static unsigned long lastTx = 0;
  if (millis() - lastTx >= 33) {
    lastTx = millis();

    float txX, txY, txH;
    bool txValid;

    portENTER_CRITICAL(&rtosMux);
    txX = posX;
    txY = posY;
    txH = odomYaw; // Broadasting encoder-derived Yaw, ignoring the IMU for position
    txValid = odomValid;
    portEXIT_CRITICAL(&rtosMux);

    if (txValid) {
      Serial.print("X:");
      Serial.print(txX, 2);
      Serial.print(",Y:");
      Serial.print(txY, 2);
      Serial.print(",H:");
      Serial.println(txH, 3);
    } else {
      Serial.print("X:INV,Y:INV,H:");
      Serial.println(txH, 3);
    }
  }
  delay(1);
}

// ==========================================
// Core 0: IMU + encoder + PID loop
// ==========================================
void ControlLoopTask(void * pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(33);

  for (;;) {
    // -------------------------
    // 1. IMU update (Purely for PID Correction)
    // -------------------------
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long currentTime = millis();
    float dt_imu = (currentTime - lastImuTime) / 1000.0f;
    lastImuTime = currentTime;
    if (dt_imu < 0.0f || dt_imu > 0.2f) { dt_imu = 0.033f; }

    portENTER_CRITICAL(&rtosMux);
    currentYaw += ((g.gyro.z - gyroZOffset) * 57.2958f * dt_imu);
    while (currentYaw > 180.0f) currentYaw -= 360.0f;
    while (currentYaw < -180.0f) currentYaw += 360.0f;
    portEXIT_CRITICAL(&rtosMux);

    // -------------------------
    // 2. Encoder sampling
    // -------------------------
    long rawLR_tach;
    portENTER_CRITICAL(&isrMux);
    rawLR_tach = pulseCountLR;
    pulseCountLR = 0;
    portEXIT_CRITICAL(&isrMux);

    long rawFL = encFL.getCount();
    long rawFR = encFR.getCount();
    long rawRR = encRR.getCount();
    encFL.clearCount();
    encFR.clearCount();
    encRR.clearCount();

    int leftDirectionSign = 1;
    if (rawFL < 0) { leftDirectionSign = 1; }
    else if (rawFL > 0) { leftDirectionSign = -1; } 
    else { leftDirectionSign = (digitalRead(LR_IN3) == HIGH) ? 1 : -1; }

    const float lrTrimFactor = 1.12f;
    long virtualSignedLR = (long)((rawLR_tach * lrTrimFactor) * leftDirectionSign);
    float signedPpsFL = rawFL * 30.3f;
    float signedPpsFR = rawFR * 30.3f;
    float signedPpsLR = virtualSignedLR * 30.3f;
    float signedPpsRR = rawRR * 30.3f;

    // -------------------------
    // 3. Encoder Kinematics (Overriding IMU for mapping)
    // -------------------------
    float avgLeftPPS = (signedPpsFL + signedPpsLR) / 2.0f;
    float avgRightPPS = (signedPpsFR + signedPpsRR) / 2.0f;
    
    // Distances covered in this 33ms window
    float distLeft = avgLeftPPS * mmPerPulse * 0.033f; 
    float distRight = avgRightPPS * mmPerPulse * 0.033f;
    float distCenter = (distLeft + distRight) / 2.0f;
    
    // Hardware derived yaw change calculation
    float deltaYaw = (distRight - distLeft) / trackWidthMM;

    portENTER_CRITICAL(&rtosMux);
    odomYaw += deltaYaw;
    
    // Clamp odomYaw between -PI and PI
    while (odomYaw > PI) odomYaw -= TWO_PI;
    while (odomYaw < -PI) odomYaw += TWO_PI;

    // Apply movement over absolute vector
    posX += distCenter * cos(odomYaw);
    posY += distCenter * sin(odomYaw);

    float localCurrentYaw = currentYaw; // Maintain IMU just for error tracking
    bool localIsTurning = isTurning;
    float localTargetYaw = targetYaw;
    DriveState localPendingState = pendingState;
    float localActiveTarget = activeTargetPPS;
    portEXIT_CRITICAL(&rtosMux);

    // -------------------------
    // 4. Absolute wheel speeds
    // -------------------------
    float ppsFL = abs(signedPpsFL);
    float ppsFR = abs(signedPpsFR);
    float ppsLR = abs(signedPpsLR);
    float ppsRR = abs(signedPpsRR);

    // -------------------------
    // 5. Safe Transition & Ramping Logic
    // -------------------------
    float localTargetPPS = 0.0f;
    if (currentState != localPendingState) {
      localTargetPPS = 0.0f;
      if (currentRampedPPS <= 0.0f) {
        currentRampedPPS = 0.0f;
        currentState = localPendingState;
        intFL = intFR = intLR = intRR = 0.0f;
        integralYaw = 0.0f;

        if (currentState == FWD)        setDirections(true, true, true, true);
        else if (currentState == REV)   setDirections(false, false, false, false);
        else if (currentState == LEFT)  setDirections(false, true, false, true);
        else if (currentState == RIGHT) setDirections(true, false, true, false);
      }
    } else {
      localTargetPPS = localActiveTarget;
    }

    if (currentRampedPPS < localTargetPPS) {
      currentRampedPPS += maxAccelPerLoop;
      if (currentRampedPPS > localTargetPPS) currentRampedPPS = localTargetPPS;
    } else if (currentRampedPPS > localTargetPPS) {
      currentRampedPPS -= maxAccelPerLoop;
      if (currentRampedPPS < localTargetPPS) currentRampedPPS = localTargetPPS;
    }

    // -------------------------
    // 6. Motor control
    // -------------------------
    if (currentRampedPPS == 0.0f && currentState == STOPPED) {
      stopAllMotors();
      intFL = intFR = intLR = intRR = 0.0f;
      integralYaw = 0.0f;
    } else {
      float targetLeft = currentRampedPPS;
      float targetRight = currentRampedPPS;
      
      // Use IMU for instantaneous PID balancing
      if (!localIsTurning) {
        float errorYaw = localTargetYaw - localCurrentYaw;
        while (errorYaw > 180.0f) errorYaw -= 360.0f;
        while (errorYaw < -180.0f) errorYaw += 360.0f;

        integralYaw += errorYaw;
        integralYaw = constrain(integralYaw, -1000.0f, 1000.0f);

        float yawCorrection = (Kp_yaw * errorYaw) + (Ki_yaw * integralYaw);

        targetLeft = currentRampedPPS - yawCorrection;
        targetRight = currentRampedPPS + yawCorrection;
      } else {
        integralYaw = 0.0f;
      }

      targetLeft = constrain(targetLeft, 0.0f, 80000.0f);
      targetRight = constrain(targetRight, 0.0f, 80000.0f);

      int pwmFL = getPID(targetLeft, ppsFL, intFL);
      int pwmFR = getPID(targetRight, ppsFR, intFR);
      int pwmLR = getPID(targetLeft, ppsLR, intLR);
      int pwmRR = getPID(targetRight, ppsRR, intRR);

      writeMotorPWM(pwmFL, pwmFR, pwmLR, pwmRR);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ==========================================
// Helpers
// ==========================================
float calibrateGyroZOffset() {
  sensors_event_t a, g, temp;
  const int samples = 1500;
  float sumZ = 0.0f;

  delay(1000);
  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&a, &g, &temp);
    sumZ += g.gyro.z;
    delay(2);
  }

  return sumZ / samples;
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'W' || c == 'w' ||
        c == 'A' || c == 'a' ||
        c == 'S' || c == 's' ||
        c == 'D' || c == 'd' ||
        c == '0' ||
        c == 'Z' || c == 'z') {
      processCommand(String(c));
      serialLine = "";
    }
    else if (c == '\n' || c == '\r') {
      if (serialLine.length() > 0) {
        processCommand(serialLine);
        serialLine = "";
      }
    }
    else {
      serialLine += c;
      if (serialLine.length() > 64) {
        serialLine = "";
      }
    }
  }
}

void processCommand(String cmd) {
  cmd.trim();

  portENTER_CRITICAL(&rtosMux);
  if (cmd.equalsIgnoreCase("W")) {
    targetYaw = currentYaw;
    isTurning = false;
    activeTargetPPS = 80000.0f;
    pendingState = FWD;
  }
  else if (cmd.equalsIgnoreCase("S")) {
    targetYaw = currentYaw;
    isTurning = false;
    activeTargetPPS = 80000.0f;
    pendingState = REV;
  }
  else if (cmd.equalsIgnoreCase("A")) {
    isTurning = true;
    activeTargetPPS = 40000.0f;
    pendingState = LEFT;
  }
  else if (cmd.equalsIgnoreCase("D")) {
    isTurning = true;
    activeTargetPPS = 40000.0f;
    pendingState = RIGHT;
  }
  else if (cmd.equals("0")) {
    activeTargetPPS = 0.0f;
    pendingState = STOPPED;
  }
  else if (cmd.equalsIgnoreCase("Z")) {
    currentYaw = 0.0f;
    targetYaw = 0.0f;
    integralYaw = 0.0f;
  }
  // NEW FUSED POSITION COMMAND PARSER: P<X>,<Y>,<YAW>
  else if (cmd.startsWith("P") || cmd.startsWith("p")) {
    int commaIndex1 = cmd.indexOf(',');
    int commaIndex2 = cmd.indexOf(',', commaIndex1 + 1);
    
    if (commaIndex1 > 1 && commaIndex2 > commaIndex1) {
      String xStr = cmd.substring(1, commaIndex1);
      String yStr = cmd.substring(commaIndex1 + 1, commaIndex2);
      String yawStr = cmd.substring(commaIndex2 + 1);

      posX = xStr.toFloat();
      posY = yStr.toFloat();
      odomYaw = yawStr.toFloat();
      odomValid = true;
    }
  }

  portEXIT_CRITICAL(&rtosMux);
}

void setDirections(bool fl, bool fr, bool lr, bool rr) {
  digitalWrite(FL_IN1, !fl); digitalWrite(FL_IN2,  fl);
  digitalWrite(FR_IN1,  fr); digitalWrite(FR_IN2, !fr);
  digitalWrite(LR_IN3,  lr); digitalWrite(LR_IN4, !lr);
  digitalWrite(RR_IN3,  rr); digitalWrite(RR_IN4, !rr);
}

int getPID(float target, float current, float &i) {
  float err = target - current;
  i += err;
  i = constrain(i, -1000.0f, 1000.0f);

  float output = (Kp * err) + (Ki * i);
  output = constrain(output, 0.0f, 255.0f);
  return (int)output;
}

void writeMotorPWM(int fl, int fr, int lr, int rr) {
  fl = constrain(fl, 0, 255);
  fr = constrain(fr, 0, 255);
  lr = constrain(lr, 0, 255);
  rr = constrain(rr, 0, 255);

  ledcWrite(FL_PWM, fl); ledcWrite(FR_PWM, fr);
  ledcWrite(LR_PWM, lr); ledcWrite(RR_PWM, rr);
}

void stopAllMotors() {
  ledcWrite(FL_PWM, 0); ledcWrite(FR_PWM, 0);
  ledcWrite(LR_PWM, 0); ledcWrite(RR_PWM, 0);
}