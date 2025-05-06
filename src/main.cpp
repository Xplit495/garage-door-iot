#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.h>
#include <ESP32Servo.h>

constexpr int RED_LED = 19;
constexpr int ORANGE_LED = 16;
constexpr int GREEN_LED = 15;
constexpr int SWITCH_OPEN = 26;
constexpr int SWITCH_CLOSE = 27;
constexpr int BUTTON = 12;
constexpr int BUZZER = 13;
constexpr int IR_RECEIVER = 25;
constexpr int SERVO_PIN = 18;

constexpr unsigned long OPEN_PRESS_MIN = 1700;
constexpr unsigned long OPEN_PRESS_MAX = 2300;
constexpr unsigned long LOCK_PRESS_MIN = 4700;
constexpr unsigned long LOCK_PRESS_MAX = 5300;
constexpr unsigned long BUTTON_FEEDBACK_TIME = 500;
constexpr uint16_t BUZZER_FREQUENCY = 262;
constexpr uint8_t IR_CODE_TOGGLE_LOCK = 226;
constexpr uint8_t IR_CODE_TOGGLE_DOOR = 162;
constexpr uint16_t MOTOR_STEP_DELAY = 26;

enum State {
  CLOSING,
  CLOSED,
  LOCKED,
  OPENING,
  OPEN,
  ERROR
};

const char* STATE_NAMES[] = {
  "CLOSING",
  "CLOSED",
  "LOCKED",
  "OPENING",
  "OPEN",
  "ERROR"
};

State doorState = CLOSED;

bool buttonWasPressedBefore = false;

unsigned long pressedTime = 0;
unsigned long releasedTime = 0;

uint8_t redLedBlinkCounter = 0;

char lastDisplayedText[32] = "";
Adafruit_SSD1306 display(128, 64, &Wire);

Servo servo;


void displayCenteredText(const char* text) {
  if (strcmp(lastDisplayedText, text) != 0) {
    display.clearDisplay();

    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
    int x = (display.width() - w) / 2;
    int y = (display.height() - h) / 2;

    display.setCursor(x, y);
    display.println(text);
    display.display();

    strncpy(lastDisplayedText, text, sizeof(lastDisplayedText) - 1);
    lastDisplayedText[sizeof(lastDisplayedText) - 1] = '\0';
  }
}

bool checkError() {
  return
    (digitalRead(SWITCH_OPEN) == LOW && digitalRead(SWITCH_CLOSE) == LOW) ||
    (digitalRead(SWITCH_OPEN) == HIGH && digitalRead(SWITCH_CLOSE) == HIGH) ||
    (doorState == CLOSED && (digitalRead(SWITCH_CLOSE) != LOW || digitalRead(SWITCH_OPEN) != HIGH)) ||
    (doorState == OPEN && (digitalRead(SWITCH_OPEN) != LOW || digitalRead(SWITCH_CLOSE) != HIGH));
}

void enterErrorMode() {
  doorState = ERROR;
  displayCenteredText(STATE_NAMES[doorState]);

  digitalWrite(RED_LED, LOW);
  digitalWrite(ORANGE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  while (true)
  {
    tone(BUZZER, BUZZER_FREQUENCY);
    digitalWrite(RED_LED, HIGH);
    delay(100);
    digitalWrite(RED_LED, LOW);
    digitalWrite(ORANGE_LED, HIGH);
    delay(100);
    digitalWrite(ORANGE_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    delay(100);
    digitalWrite(GREEN_LED, LOW);
    noTone(BUZZER);
  }
}

void handleLockedStateLED() {
  if (doorState == LOCKED) {
    if (redLedBlinkCounter == 5) {
      redLedBlinkCounter = 0;
      digitalWrite(RED_LED, HIGH);
      delay(300);
      digitalWrite(RED_LED, LOW);
    } else {
      redLedBlinkCounter++;
    }
  }
}

void handleLockState() {
  if (doorState == CLOSED) {
    doorState = LOCKED;
    digitalWrite(RED_LED, LOW);
  } else if (doorState == LOCKED) {
    doorState = CLOSED;
    digitalWrite(RED_LED, HIGH);
  }
}

void activateMotor() {
  if (doorState == OPENING) {
    for (int angle = 0; angle <= 180; angle++) {
      servo.write(angle);
      delay(MOTOR_STEP_DELAY);
  }
  } else if (doorState == CLOSING) {
    for (int angle = 180; angle >= 0; angle--) {
      servo.write(angle);
      delay(MOTOR_STEP_DELAY);
    }
  }
}

void handleDoorOpening() {
  digitalWrite(RED_LED, LOW);
  digitalWrite(ORANGE_LED, HIGH);
  doorState = OPENING;
  displayCenteredText(STATE_NAMES[doorState]);
  activateMotor();

  if (digitalRead(SWITCH_OPEN) == LOW && digitalRead(SWITCH_CLOSE) == HIGH) {
    doorState = OPEN;
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(ORANGE_LED, LOW);
  } else {
    enterErrorMode();
  }
}

void handleDoorClosing() {
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(ORANGE_LED, HIGH);
  doorState = CLOSING;
  displayCenteredText(STATE_NAMES[doorState]);
  activateMotor();

  if (digitalRead(SWITCH_OPEN) == HIGH && digitalRead(SWITCH_CLOSE) == LOW) {
    doorState = CLOSED;
    digitalWrite(RED_LED, HIGH);
    digitalWrite(ORANGE_LED, LOW);
  } else {
    enterErrorMode();
  }
}

void toggleDoorPosition() {
  if (doorState == LOCKED) {
    tone(BUZZER, BUZZER_FREQUENCY);
    displayCenteredText("LOCKED!!");
    delay(3000);
    noTone(BUZZER);
  }
  else if (doorState == CLOSED) {
    handleDoorOpening();
  } else if (doorState == OPEN) {
    handleDoorClosing();
  }
}

void handleIRSignal()
{
  if (IrReceiver.decode()) {
    switch (IrReceiver.decodedIRData.command){
      case IR_CODE_TOGGLE_DOOR:
        toggleDoorPosition();
        break;
      case IR_CODE_TOGGLE_LOCK:
        handleLockState();
        break;
      default:
        Serial.println("Unknown command");
    }
    IrReceiver.resume();
  }
}

void handleButtonPress() {
  if (digitalRead(BUTTON) == LOW) {
    if (!buttonWasPressedBefore) {
      buttonWasPressedBefore = true;
      pressedTime = millis();

      digitalWrite(ORANGE_LED, HIGH);
      delay(BUTTON_FEEDBACK_TIME);
      digitalWrite(ORANGE_LED, LOW);
    }
  }
  else if (digitalRead(BUTTON) == HIGH && buttonWasPressedBefore) {
    releasedTime = millis();
    unsigned long rawDuration = releasedTime - pressedTime;

    Serial.print("Button pressed for: ");
    Serial.println(rawDuration);

    if (rawDuration > OPEN_PRESS_MIN && rawDuration < OPEN_PRESS_MAX) {
      toggleDoorPosition();
    }
    else if (rawDuration > LOCK_PRESS_MIN && rawDuration < LOCK_PRESS_MAX) {
      handleLockState();
    }

    buttonWasPressedBefore = false;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(RED_LED, OUTPUT);
  pinMode(ORANGE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(SWITCH_OPEN, INPUT_PULLUP);
  pinMode(SWITCH_CLOSE, INPUT_PULLUP);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextSize(2);
  display.setTextColor(WHITE);

  IrReceiver.begin(IR_RECEIVER);

  servo.attach(SERVO_PIN);

  handleDoorClosing(); // Initialize door state to closed
}

void loop() {
  if (checkError()) {
    enterErrorMode();
  }

  handleIRSignal();

  handleButtonPress();

  handleLockedStateLED();

  displayCenteredText(STATE_NAMES[doorState]);

  delay(100);
}