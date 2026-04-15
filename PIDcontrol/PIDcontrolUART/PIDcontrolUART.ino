// ================= MOTOR L =================
#define ENB 10
#define IN3 5
#define IN4 4
#define ENC_A1 3
#define ENC_B1 19

// ================= MOTOR R ==================
#define ENA 9
#define IN1 7
#define IN2 6
#define ENC_A2 2
#define ENC_B2 18

// ================= PARAMETERS ======================
const float MAX_RPM = 130.0;
const int   PWM_MAX = 255;
const float PULSES_PER_REV = 514.8;
const float RPM_FILTER_ALPHA = 0.3;
const float WHEEL_RADIUS = 0.033; // ms

const float V_TO_RPM = 60.0 / (2 * PI * WHEEL_RADIUS);
const unsigned long UART_TIMEOUT = 200;  // ms 
unsigned long lastUARTTime = 0;

const unsigned long CONTROL_INTERVAL_MS = 30;

// Feedforward
const float KFF = 0.9;

// Balance
float K_balance = 0.1;

// Ramp
const float RAMP_STEP = 11.0;

// ================= ENCODER ======================
volatile long encCount1 = 0;
volatile long encCount2 = 0;

float rpm1Smooth = 0;
float rpm2Smooth = 0;

// ================= PID ======================
struct PID {
  float Kp, Ki, Kd;
  float e1, e2;
  float u;
};

// PID đã giảm để ổn định
PID pid1 = {1.0, 0.15, 0.01, 0, 0, 0};
PID pid2 = {1.0, 0.15, 0.01, 0, 0, 0};

// // ===== TARGET =====
float targetRPM1_cmd = 0;
float targetRPM2_cmd = 0;

float targetRPM1 = 50;
float targetRPM2 = 50;

int pwm1 = 0, pwm2 = 0;

unsigned long lastControlTime = 0;

void resetPID(PID &pid) {
  pid.e1 = 0;
  pid.e2 = 0;
  pid.u = 0;
}

// ==================================================
void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(ENC_A1, INPUT_PULLUP);  pinMode(ENC_B1, INPUT_PULLUP);
  pinMode(ENC_A2, INPUT_PULLUP);  pinMode(ENC_B2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), encoder2ISR, CHANGE);

  lastControlTime = millis();
}

// ==================================================
void loop() {
  unsigned long now = millis();

  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    unsigned long dt = now - lastControlTime;
    lastControlTime += CONTROL_INTERVAL_MS;
    float dt_s = dt / 1000.0;

    // ===== Read encoder =====
    noInterrupts();
    long c1 = encCount1;
    long c2 = encCount2;
    encCount1 = 0;
    encCount2 = 0;
    interrupts();

    float rpm1 = computeRPM(c1, dt);
    float rpm2 = computeRPM(c2, dt);

    bool uartTimeout = (millis() - lastUARTTime > UART_TIMEOUT);
    if (uartTimeout) {
      targetRPM1_cmd = 0;
      targetRPM2_cmd = 0;

      targetRPM1 = 0;
      targetRPM2 = 0;

      resetPID(pid1);
      resetPID(pid2);

      setMotor1(0);
      setMotor2(0);
    }

    if (abs(rpm1Smooth) < 1 && abs(targetRPM1) > 10) {
      resetPID(pid1);
      pwm1 = 0;
    }

    if (abs(rpm2Smooth) < 1 && abs(targetRPM2) > 10) {
      resetPID(pid2);
      pwm2 = 0;
    }

    // ===== Filter =====
    rpm1Smooth = rpm1Smooth * (1.0 - RPM_FILTER_ALPHA) + rpm1 * RPM_FILTER_ALPHA;
    rpm2Smooth = rpm2Smooth * (1.0 - RPM_FILTER_ALPHA) + rpm2 * RPM_FILTER_ALPHA;

    // ===== Ramp target (mượt, không đứng) =====
    targetRPM1 += constrain(targetRPM1_cmd - targetRPM1, -RAMP_STEP, RAMP_STEP);
    targetRPM2 += constrain(targetRPM2_cmd - targetRPM2, -RAMP_STEP, RAMP_STEP);

    // ===== Balance =====
    float diff = rpm1Smooth - rpm2Smooth;
    float t1 = targetRPM1 - K_balance * diff;
    float t2 = targetRPM2 + K_balance * diff;

    // ===== PID =====
    if (!uartTimeout) {
    pwm1 = computePID(pid1, t1, rpm1Smooth, dt_s);
    pwm2 = computePID(pid2, t2, rpm2Smooth, dt_s);

    // ===== Output =====
    setMotor1(pwm1);
    setMotor2(pwm2);
    }

    // ===== Debug (dùng Python plot) =====
    Serial.print(targetRPM1); Serial.print("   ");
    // Serial.print(millis());   Serial.print("   ");
    Serial.print(rpm1Smooth); Serial.print("   ");
    Serial.print(rpm2Smooth); Serial.print("   ");
    Serial.println(diff);
  }

  // ===== SERIAL INPUT (FORMAT: <vL,vR>) =====
  static String inputString = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '<') {
      inputString = "";
    }
    else if (c == '>') {
      int commaIndex = inputString.indexOf(',');

      if (commaIndex > 0) {
        float v_left  = inputString.substring(0, commaIndex).toFloat();
        float v_right = inputString.substring(commaIndex + 1).toFloat();

        // ===== Convert m/s → RPM =====
        float rpmL = v_left * V_TO_RPM;
        float rpmR = v_right * V_TO_RPM;

        // ===== Clamp =====
        targetRPM1_cmd = constrain(rpmL, -MAX_RPM, MAX_RPM);
        targetRPM2_cmd = constrain(rpmR, -MAX_RPM, MAX_RPM);

        // ===== RESET TIMEOUT =====
        lastUARTTime = millis();

        // ===== DEBUG =====
        Serial.print("CMD: ");
        Serial.print(v_left);
        Serial.print(" | ");
        Serial.print(v_right);
        Serial.print(" || RPM_CMD: ");
        Serial.print(targetRPM1_cmd);
        Serial.print(" | ");
        Serial.println(targetRPM2_cmd);
      }
    }
    else {
      inputString += c;
    }
  }
}

// ================= PID ====================
float computePID(PID &pid, float target, float current, float dt) {
  float e = target - current;

  float a1 = pid.Kp + pid.Ki * dt + pid.Kd / dt;
  float a2 = -pid.Kp - 2.0 * pid.Kd / dt;
  float a3 = pid.Kd / dt;

  float du = a1 * e + a2 * pid.e1 + a3 * pid.e2;
  pid.u += du;

  float ff = KFF * target;
  float output = pid.u + ff;

  pid.e2 = pid.e1;
  pid.e1 = e;

  return constrain(output, -PWM_MAX, PWM_MAX);
}

// ================= ISR =============================
void encoder1ISR() {
  bool A = digitalRead(ENC_A1);
  bool B = digitalRead(ENC_B1);
  encCount1 += (A == B) ? 1 : -1;
}

void encoder2ISR() {
  bool A = digitalRead(ENC_A2);
  bool B = digitalRead(ENC_B2);
  encCount2 += (A == B) ? -1 : 1;
}

// ================= MOTOR ===========================
void setMotor1(int pwm) {
  digitalWrite(IN3, pwm >= 0);
  digitalWrite(IN4, pwm < 0);
  analogWrite(ENB, abs(pwm));
}

void setMotor2(int pwm) {
  digitalWrite(IN1, pwm < 0);
  digitalWrite(IN2, pwm >= 0);
  analogWrite(ENA, abs(pwm));
}

// ================= RPM =============================
float computeRPM(long dc, unsigned long dt) {
  return (dc * 60000.0) / (PULSES_PER_REV * dt);
}
