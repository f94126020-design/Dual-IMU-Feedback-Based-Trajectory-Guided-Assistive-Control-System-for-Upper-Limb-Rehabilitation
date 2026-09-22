#include <Arduino.h>
#include <ctype.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// =============================================================================
// ESP32 Embedded Motor Safety Node
//
// Hardware:
//   ESP32 + 3 x BTS7960
//   Motor 1: biceps
//   Motor 2: triceps
//   Motor 3: deltoid
//
// Raspberry Pi -> ESP32:
//   ARM
//   PWM,p1,p2,p3      signed PWM, each value must be -255..255
//   STOP               immediate normal stop, return to IDLE
//   ESTOP              immediate latched emergency stop
//   CLEAR              clear TIMEOUT/ESTOP fault, return to IDLE
//   PING
//   STATUS
//   ZERO               reserved for future encoders
//
// ESP32 -> Raspberry Pi:
//   READY,ESP32_MOTOR_SAFETY_NODE,V1
//   OK,...
//   ERR,...
//   FB,0,0,0,0.00,0.00,0.00,pwm1,pwm2,pwm3
//   STATE,state,command_age_ms,target1,target2,target3,applied1,applied2,applied3
//
// Safety principles:
//   1. Power-up always starts in IDLE with all drivers disabled.
//   2. PWM is rejected until ARM is received.
//   3. Only a strictly valid PWM command refreshes the 300 ms command timeout.
//   4. Normal PWM changes use a 10 ms ramp task.
//   5. STOP, ESTOP and TIMEOUT bypass the ramp and stop immediately.
//   6. ESTOP is latched. CLEAR then ARM are required before PWM can resume.
//   7. Encoder interrupts are disabled because encoders are not installed.
// =============================================================================

constexpr uint8_t MOTOR_COUNT = 3;

// PWM hardware configuration.
constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;
constexpr int PWM_LIMIT = 255;

// Deterministic application-level scheduling.
constexpr uint32_t MOTOR_UPDATE_INTERVAL_MS = 10;   // 100 Hz applied PWM update
constexpr uint32_t FEEDBACK_INTERVAL_MS = 50;       // 20 Hz legacy-compatible FB
constexpr uint32_t STATE_INTERVAL_MS = 250;         // 4 Hz detailed state report
constexpr uint32_t COMMAND_TIMEOUT_MS = 300;

// Final embedded safety ramp. The Raspberry Pi may apply a slower therapeutic
// ramp; this ESP32 ramp acts as a final guard against abrupt command jumps.
constexpr int PWM_RAMP_STEP_PER_TICK = 5;

// No encoder is installed in the current system.
constexpr bool ENABLE_ENCODERS = false;

struct MotorPins {
  uint8_t rpwm;
  uint8_t lpwm;
  uint8_t ren;
  uint8_t len;
};

// Pin assignment retained from the previously tested three-motor wiring.
const MotorPins MOTOR_PINS[MOTOR_COUNT] = {
  {25, 26, 27, 14},  // Motor 1: biceps
  {32, 33, 13, 23},  // Motor 2: triceps
  {18, 19, 21, 22},  // Motor 3: deltoid
};

// Arduino-ESP32 Core 2.x uses LEDC channel numbers. Core 3.x writes by pin.
const uint8_t RPWM_CHANNEL[MOTOR_COUNT] = {0, 2, 4};
const uint8_t LPWM_CHANNEL[MOTOR_COUNT] = {1, 3, 5};

enum class SafetyState : uint8_t {
  IDLE,
  ARMED,
  ACTIVE,
  TIMEOUT_STOP,
  EMERGENCY_STOP,
};

SafetyState safetyState = SafetyState::IDLE;

int targetPwm[MOTOR_COUNT] = {0, 0, 0};
int appliedPwm[MOTOR_COUNT] = {0, 0, 0};

uint32_t lastValidCommandTime = 0;
uint32_t lastMotorUpdateTime = 0;
uint32_t lastFeedbackTime = 0;
uint32_t lastStateReportTime = 0;
bool hasValidCommandTime = false;

constexpr size_t SERIAL_LINE_CAPACITY = 96;
char serialLine[SERIAL_LINE_CAPACITY] = {0};
size_t serialLineLength = 0;
bool serialLineOverflow = false;

// =============================================================================
// Utility
// =============================================================================

const char *stateName(SafetyState state) {
  switch (state) {
    case SafetyState::IDLE:
      return "IDLE";
    case SafetyState::ARMED:
      return "ARMED";
    case SafetyState::ACTIVE:
      return "ACTIVE";
    case SafetyState::TIMEOUT_STOP:
      return "TIMEOUT_STOP";
    case SafetyState::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
  }
  return "UNKNOWN";
}

int clampPwm(int value) {
  if (value > PWM_LIMIT) return PWM_LIMIT;
  if (value < -PWM_LIMIT) return -PWM_LIMIT;
  return value;
}

bool anyNonZero(const int values[MOTOR_COUNT]) {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (values[i] != 0) return true;
  }
  return false;
}

int approachValue(int current, int target, int step) {
  if (current < target) {
    const int next = current + step;
    return next > target ? target : next;
  }
  if (current > target) {
    const int next = current - step;
    return next < target ? target : next;
  }
  return current;
}

void trimLine(char *line) {
  char *start = line;
  while (*start != '\0' && isspace(static_cast<unsigned char>(*start))) {
    ++start;
  }

  if (start != line) {
    memmove(line, start, strlen(start) + 1);
  }

  size_t length = strlen(line);
  while (length > 0 &&
         isspace(static_cast<unsigned char>(line[length - 1]))) {
    line[length - 1] = '\0';
    --length;
  }
}

void uppercaseCopy(const char *source, char *destination, size_t capacity) {
  if (capacity == 0) return;

  size_t i = 0;
  for (; source[i] != '\0' && i + 1 < capacity; ++i) {
    destination[i] =
        static_cast<char>(toupper(static_cast<unsigned char>(source[i])));
  }
  destination[i] = '\0';
}

// =============================================================================
// PWM compatibility layer for Arduino-ESP32 Core 2.x and 3.x
// =============================================================================

bool attachPwmPin(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  return ledcAttach(pin, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
#else
  const double actualFrequency =
      ledcSetup(channel, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  if (actualFrequency <= 0.0) return false;
  ledcAttachPin(pin, channel);
  return true;
#endif
}

void writePwmPin(uint8_t pin, uint8_t channel, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

void writeRpwm(uint8_t motorIndex, uint32_t duty) {
  writePwmPin(
      MOTOR_PINS[motorIndex].rpwm,
      RPWM_CHANNEL[motorIndex],
      duty);
}

void writeLpwm(uint8_t motorIndex, uint32_t duty) {
  writePwmPin(
      MOTOR_PINS[motorIndex].lpwm,
      LPWM_CHANNEL[motorIndex],
      duty);
}

// =============================================================================
// Motor hardware
// =============================================================================

void disableDriver(uint8_t motorIndex) {
  writeRpwm(motorIndex, 0);
  writeLpwm(motorIndex, 0);
  digitalWrite(MOTOR_PINS[motorIndex].ren, LOW);
  digitalWrite(MOTOR_PINS[motorIndex].len, LOW);
}

void enableDriver(uint8_t motorIndex) {
  digitalWrite(MOTOR_PINS[motorIndex].ren, HIGH);
  digitalWrite(MOTOR_PINS[motorIndex].len, HIGH);
}

void applyMotorHardware(uint8_t motorIndex, int signedPwm) {
  signedPwm = clampPwm(signedPwm);

  if (signedPwm == 0) {
    disableDriver(motorIndex);
    return;
  }

  enableDriver(motorIndex);

  if (signedPwm > 0) {
    // Always clear the opposite input before applying the active input.
    writeLpwm(motorIndex, 0);
    writeRpwm(motorIndex, static_cast<uint32_t>(signedPwm));
  } else {
    writeRpwm(motorIndex, 0);
    writeLpwm(motorIndex, static_cast<uint32_t>(-signedPwm));
  }
}

void applyAllMotorHardware() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    applyMotorHardware(i, appliedPwm[i]);
  }
}

void immediateStopAndDisable() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    targetPwm[i] = 0;
    appliedPwm[i] = 0;
    disableDriver(i);
  }
}

bool setupMotorHardware() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    pinMode(MOTOR_PINS[i].ren, OUTPUT);
    pinMode(MOTOR_PINS[i].len, OUTPUT);
    digitalWrite(MOTOR_PINS[i].ren, LOW);
    digitalWrite(MOTOR_PINS[i].len, LOW);

    const bool rpwmAttached =
        attachPwmPin(MOTOR_PINS[i].rpwm, RPWM_CHANNEL[i]);
    const bool lpwmAttached =
        attachPwmPin(MOTOR_PINS[i].lpwm, LPWM_CHANNEL[i]);

    if (!rpwmAttached || !lpwmAttached) {
      immediateStopAndDisable();
      Serial.printf("ERR,MOTOR_%u_PWM_INIT_FAILED\n", i + 1);
      return false;
    }

    disableDriver(i);
  }

  return true;
}

// =============================================================================
// Safety state transitions
// =============================================================================

void enterIdle() {
  immediateStopAndDisable();
  safetyState = SafetyState::IDLE;
  hasValidCommandTime = false;
}

void enterArmed() {
  immediateStopAndDisable();
  safetyState = SafetyState::ARMED;
  lastValidCommandTime = millis();
  hasValidCommandTime = true;
}

void enterTimeoutStop() {
  immediateStopAndDisable();
  safetyState = SafetyState::TIMEOUT_STOP;
  hasValidCommandTime = false;
  Serial.println("FAULT,COMMAND_TIMEOUT");
}

void enterEmergencyStop() {
  immediateStopAndDisable();
  safetyState = SafetyState::EMERGENCY_STOP;
  hasValidCommandTime = false;
  Serial.println("FAULT,EMERGENCY_STOP");
}

void updateMotorRamp() {
  if (safetyState != SafetyState::ARMED &&
      safetyState != SafetyState::ACTIVE) {
    immediateStopAndDisable();
    return;
  }

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    const int current = appliedPwm[i];
    const int target = targetPwm[i];

    // A direction reversal must first ramp to zero. Since the task runs every
    // 10 ms, reaching zero here guarantees at least one zero-output tick before
    // the opposite direction is applied on the following tick.
    const bool reversing =
        (current > 0 && target < 0) || (current < 0 && target > 0);

    if (reversing) {
      appliedPwm[i] =
          approachValue(current, 0, PWM_RAMP_STEP_PER_TICK);
    } else {
      appliedPwm[i] =
          approachValue(current, target, PWM_RAMP_STEP_PER_TICK);
    }
  }

  applyAllMotorHardware();

  safetyState =
      (anyNonZero(targetPwm) || anyNonZero(appliedPwm))
          ? SafetyState::ACTIVE
          : SafetyState::ARMED;
}

void checkCommandTimeout(uint32_t now) {
  if (safetyState != SafetyState::ARMED &&
      safetyState != SafetyState::ACTIVE) {
    return;
  }

  if (!hasValidCommandTime ||
      static_cast<uint32_t>(now - lastValidCommandTime) >
          COMMAND_TIMEOUT_MS) {
    enterTimeoutStop();
  }
}

// =============================================================================
// Feedback
// =============================================================================

void sendLegacyFeedback() {
  // Encoder fields remain zero until ENABLE_ENCODERS is implemented.
  Serial.print("FB,0,0,0,0.00,0.00,0.00,");
  Serial.print(appliedPwm[0]);
  Serial.print(",");
  Serial.print(appliedPwm[1]);
  Serial.print(",");
  Serial.println(appliedPwm[2]);
}

void sendStateReport() {
  const long commandAgeMs =
      hasValidCommandTime
          ? static_cast<long>(millis() - lastValidCommandTime)
          : -1L;

  Serial.print("STATE,");
  Serial.print(stateName(safetyState));
  Serial.print(",");
  Serial.print(commandAgeMs);
  Serial.print(",");
  Serial.print(targetPwm[0]);
  Serial.print(",");
  Serial.print(targetPwm[1]);
  Serial.print(",");
  Serial.print(targetPwm[2]);
  Serial.print(",");
  Serial.print(appliedPwm[0]);
  Serial.print(",");
  Serial.print(appliedPwm[1]);
  Serial.print(",");
  Serial.println(appliedPwm[2]);
}

// =============================================================================
// Strict command parser
// =============================================================================

bool parseStrictPwmCommand(const char *command, int parsed[MOTOR_COUNT]) {
  if (strncasecmp(command, "PWM,", 4) != 0) return false;

  const char *cursor = command + 4;

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (*cursor == '\0' || isspace(static_cast<unsigned char>(*cursor))) {
      return false;
    }

    errno = 0;
    char *end = nullptr;
    const long value = strtol(cursor, &end, 10);

    if (end == cursor || errno == ERANGE ||
        value < -PWM_LIMIT || value > PWM_LIMIT) {
      return false;
    }

    parsed[i] = static_cast<int>(value);

    if (i + 1 < MOTOR_COUNT) {
      if (*end != ',') return false;
      cursor = end + 1;
    } else {
      if (*end != '\0') return false;
    }
  }

  return true;
}

void processCommand(char *command) {
  trimLine(command);
  if (command[0] == '\0') return;

  char upper[SERIAL_LINE_CAPACITY] = {0};
  uppercaseCopy(command, upper, sizeof(upper));

  // ESTOP has priority and is accepted from every state.
  if (strcmp(upper, "ESTOP") == 0) {
    enterEmergencyStop();
    Serial.println("OK,ESTOP");
    return;
  }

  // STOP is always an immediate stop. It never clears a latched ESTOP.
  if (strcmp(upper, "STOP") == 0) {
    if (safetyState == SafetyState::EMERGENCY_STOP) {
      immediateStopAndDisable();
      Serial.println("ERR,ESTOP_LATCHED");
    } else {
      enterIdle();
      Serial.println("OK,STOP,IDLE");
    }
    return;
  }

  if (strcmp(upper, "CLEAR") == 0) {
    if (safetyState == SafetyState::EMERGENCY_STOP ||
        safetyState == SafetyState::TIMEOUT_STOP) {
      enterIdle();
      Serial.println("OK,CLEAR,IDLE");
    } else {
      Serial.println("OK,CLEAR,NO_FAULT");
    }
    return;
  }

  if (strcmp(upper, "ARM") == 0) {
    if (safetyState == SafetyState::EMERGENCY_STOP) {
      Serial.println("ERR,ESTOP_LATCHED,CLEAR_REQUIRED");
      return;
    }
    if (safetyState == SafetyState::ACTIVE) {
      Serial.println("ERR,ACTIVE_SEND_STOP_FIRST");
      return;
    }

    enterArmed();
    Serial.println("OK,ARM,ARMED");
    return;
  }

  if (strcmp(upper, "PING") == 0) {
    Serial.println("OK,PONG");
    return;
  }

  if (strcmp(upper, "STATUS") == 0) {
    sendLegacyFeedback();
    sendStateReport();
    return;
  }

  if (strcmp(upper, "ZERO") == 0) {
    if (ENABLE_ENCODERS) {
      // Reserved for the future encoder implementation.
      Serial.println("OK,ZERO");
    } else {
      Serial.println("OK,ZERO,ENCODERS_DISABLED");
    }
    return;
  }

  if (strncasecmp(upper, "PWM,", 4) == 0) {
    if (safetyState == SafetyState::EMERGENCY_STOP) {
      Serial.println("ERR,ESTOP_LATCHED");
      return;
    }
    if (safetyState == SafetyState::TIMEOUT_STOP) {
      Serial.println("ERR,TIMEOUT_LATCHED,ARM_REQUIRED");
      return;
    }
    if (safetyState != SafetyState::ARMED &&
        safetyState != SafetyState::ACTIVE) {
      Serial.println("ERR,NOT_ARMED");
      return;
    }

    int parsed[MOTOR_COUNT] = {0, 0, 0};
    if (!parseStrictPwmCommand(upper, parsed)) {
      Serial.println("ERR,BAD_PWM_FORMAT");
      return;
    }

    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
      targetPwm[i] = parsed[i];
    }

    lastValidCommandTime = millis();
    hasValidCommandTime = true;

    Serial.print("OK,PWM,");
    Serial.print(targetPwm[0]);
    Serial.print(",");
    Serial.print(targetPwm[1]);
    Serial.print(",");
    Serial.println(targetPwm[2]);
    return;
  }

  Serial.print("ERR,UNKNOWN_COMMAND,");
  Serial.println(command);
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char received = static_cast<char>(Serial.read());

    if (received == '\n' || received == '\r') {
      if (serialLineOverflow) {
        Serial.println("ERR,LINE_TOO_LONG");
      } else if (serialLineLength > 0) {
        serialLine[serialLineLength] = '\0';
        processCommand(serialLine);
      }

      serialLineLength = 0;
      serialLine[0] = '\0';
      serialLineOverflow = false;
      continue;
    }

    if (serialLineOverflow) {
      continue;
    }

    if (serialLineLength + 1 < SERIAL_LINE_CAPACITY) {
      serialLine[serialLineLength++] = received;
    } else {
      serialLineOverflow = true;
    }
  }
}

// =============================================================================
// Arduino
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(800);

  if (!setupMotorHardware()) {
    safetyState = SafetyState::EMERGENCY_STOP;
    while (true) {
      immediateStopAndDisable();
      delay(100);
    }
  }

  enterIdle();

  const uint32_t now = millis();
  lastMotorUpdateTime = now;
  lastFeedbackTime = now;
  lastStateReportTime = now;

  Serial.println("READY,ESP32_MOTOR_SAFETY_NODE,V1");
  sendStateReport();
}

void loop() {
  // Serial parsing remains responsive and independent of the periodic tasks.
  readSerialCommands();

  const uint32_t now = millis();

  // Communication loss is checked every loop and stops immediately.
  checkCommandTimeout(now);

  // Execute at most one ramp step per loop. If a tick was missed, do not run
  // multiple catch-up steps because that would create an abrupt PWM jump.
  if (static_cast<uint32_t>(now - lastMotorUpdateTime) >=
      MOTOR_UPDATE_INTERVAL_MS) {
    lastMotorUpdateTime += MOTOR_UPDATE_INTERVAL_MS;

    if (static_cast<uint32_t>(now - lastMotorUpdateTime) >=
        MOTOR_UPDATE_INTERVAL_MS) {
      lastMotorUpdateTime = now;
    }

    updateMotorRamp();
  }

  if (static_cast<uint32_t>(now - lastFeedbackTime) >=
      FEEDBACK_INTERVAL_MS) {
    lastFeedbackTime = now;
    sendLegacyFeedback();
  }

  if (static_cast<uint32_t>(now - lastStateReportTime) >=
      STATE_INTERVAL_MS) {
    lastStateReportTime = now;
    sendStateReport();
  }

  // Yield to the ESP32 Arduino/FreeRTOS background tasks and watchdog.
  delay(1);
}
