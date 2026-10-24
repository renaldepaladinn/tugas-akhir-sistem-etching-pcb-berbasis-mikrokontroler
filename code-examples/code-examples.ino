// LIBRARY
#include <ezButton.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

// PIN
// ULTRASONIK
const uint8_t TRIG_PIN[] = { 22, 24, 26 };
const uint8_t ECHO_PIN[] = { 23, 25, 27 };
// FLOW SENSOR
const uint8_t FLOW_PIN = 3;
// LIMIT SWITCH
const uint8_t LIMITA_PIN = 28;
const uint8_t LIMITB_PIN = 29;
// PUSH BUTTON
const uint8_t PBSTART_PIN = 30;
const uint8_t PBSTOP_PIN = 31;
const uint8_t PBDRAIN_PIN = 32;
// POTENSIO
const uint8_t POT_PIN = A0;
// RELAY
const uint8_t RELAYA_PIN = 33;
const uint8_t RELAYB_PIN = 34;
// MOSFET
const uint8_t PWM_PIN = 7;
// H-BRIDGE
const uint8_t RPWM_PIN = 5;
const uint8_t LPWM_PIN = 6;
// OLED
const uint8_t RESET_PIN = 12;
// SERVO
const uint8_t SERVO_PIN = 4;

//SETUP AWAL
// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, RESET_PIN);
// BUTTON
ezButton btnStart(PBSTART_PIN);
ezButton btnStop(PBSTOP_PIN);
ezButton btnDrain(PBDRAIN_PIN);
ezButton lmtAtas(LIMITA_PIN);
ezButton lmtBawah(LIMITB_PIN);
// SERVO
Servo myServo;
// FLOW SENSOR
volatile unsigned long pulseCount = 0;
float volume = 400;
float targetPulsa = 0;
// SENSOR
float levelLarutanBekas = 0;
float levelAirBersih = 0;
float levelFerric = 0;
int persenBuang = 0;
int persenAir = 0;
int persenFeCl3 = 0;
unsigned long prevSensor = 0;
// REGRESI
const uint8_t JUMLAH_SENSOR = 3;
const float REGRESISONIC_M[JUMLAH_SENSOR] = { 1.0457, 1.0459, 1.0453 };
const float REGRESISONIC_C[JUMLAH_SENSOR] = { 0.0063, 0.1496, 0.302 };
// WAKTU
int waktuPerStep = 0;
long timerPerProses = 0;
long timerSisaWaktu = 0;
unsigned long prevMillis = 0;
// TOTAL PEMAKAIAN
long totalWaktuLarutan = 0;
int sisaWaktuLarutan = 0;
// STATUS
bool statusLarutan = false;
bool blinkState = false;
unsigned long prevBlink = 0;
uint8_t infoState = false;
unsigned long prevInfo = 0;
bool tampilBahanKurang = false;
unsigned long tBahanKurang = 0;
// POTENSIO
int valPot = 0;
int lastPot = 0;
// STATUS SET WAKTU
bool modeSetWaktu = false;
unsigned long prevPot = 0;
// DISPLAY WAKTU
int menit = 0;
int detik = 0;
int16_t x1, y1;
uint16_t w, h;
int persenBuangDisplay = 0;
int persenAirDisplay = 0;
int persenFeCl3Display = 0;
// PAUSE LIFTER
bool prosesNaikPause = false;
unsigned long tLifter = 0;
uint8_t stepLifterTurun = 1;
//DRAIN
static bool mulaiDrain = false;
static bool tampilWarning = true;
static unsigned long prevDrain = 0;
static int timerDrain = 0;
uint8_t stepDrain = 1;
unsigned long tWarningDrain = 0;
bool autoDrainMode = false;
// STATE
enum Proses {
  PROSES_MONITORING,
  PROSES_SOLUTION,
  PROSES_LIFTER_TURUN,
  PROSES_ETCHING,
  PROSES_LIFTER_NAIK,
  PROSES_PAUSE,
  PROSES_FINISH,
  PROSES_DRAIN
};

Proses currentProses = PROSES_MONITORING;

// PROSES 2
uint8_t stepLarutan = 1;
unsigned long tServo = 0;
unsigned long tMosfet = 0;
// PROSES 5
uint8_t stepLifterNaik = 1;
unsigned long tLifterNaik = 0;
// PROSES 3
bool etchingPause = false;
bool delayNaik = false;
unsigned long tDelayNaik = 0;

// INTERRUPT FLOW SENSOR
void countPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(9600);
  // ULTRASONIK
  for (int i = 0; i < JUMLAH_SENSOR; i++) {
    pinMode(TRIG_PIN[i], OUTPUT);
    pinMode(ECHO_PIN[i], INPUT);
  }
  // FLOW SENSOR
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(FLOW_PIN),
    countPulse,
    RISING);
  // BUTTON
  pinMode(PBSTART_PIN, INPUT_PULLUP);
  pinMode(PBSTOP_PIN, INPUT_PULLUP);
  pinMode(PBDRAIN_PIN, INPUT_PULLUP);
  pinMode(LIMITA_PIN, INPUT_PULLUP);
  pinMode(LIMITB_PIN, INPUT_PULLUP);
  btnStart.setDebounceTime(50);
  btnStop.setDebounceTime(50);
  btnDrain.setDebounceTime(50);
  lmtAtas.setDebounceTime(50);
  lmtBawah.setDebounceTime(50);
  // OUTPUT
  pinMode(RELAYA_PIN, OUTPUT);
  pinMode(RELAYB_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  // KONDISI AWAL
  digitalWrite(RELAYA_PIN, HIGH);
  digitalWrite(RELAYB_PIN, HIGH);
  analogWrite(PWM_PIN, 0);
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
  // SERVO
  myServo.attach(SERVO_PIN);
  myServo.writeMicroseconds(1500);
  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 gagal"));
    while (1)
      ;
  }
  lastPot = analogRead(POT_PIN);
  display.clearDisplay();
  // TARGET FLOW
  targetPulsa = (4.9547 * volume) + 95.156;
  judulTA();
  delay(3000);
}

void loop() {
  btnStart.loop();
  btnStop.loop();
  btnDrain.loop();
  lmtAtas.loop();
  lmtBawah.loop();
  // ----BLINK----
  if (millis() - prevBlink >= 500) {  // untuk tampilan monitoring error
    prevBlink = millis();
    blinkState = !blinkState;
  }
  if (millis() - prevInfo >= 5000) {  // untuk info bagian bawah di monitoring
    prevInfo = millis();
    infoState++;
    if (infoState > 2) {
      infoState = 0;
    }
  }
  // PROSES
  switch (currentProses) {
    // PROSES 1 MONITORING
    case PROSES_MONITORING:
      prosesMonitoring();
      break;
    // PROSES 2 LARUTAN
    case PROSES_SOLUTION:
      prosesSolution();
      break;
    // PROSES 3 LIFTER TURUN
    case PROSES_LIFTER_TURUN:
      lifterTurun();
      break;
    // PROSES 4 PROSES ETCHING
    case PROSES_ETCHING:
      prosesEtching();
      break;
    // PROSES 5 LIFTER NAIK
    case PROSES_LIFTER_NAIK:
      lifterNaik();
      break;
    // PROSES 6 JIKA TERJADI INTRUPSI STOP
    case PROSES_PAUSE:
      prosesPause();
      break;
    // PROSES 7 FINISH
    case PROSES_FINISH:
      prosesFinish();
      break;
    // PROSES 8 DRAIN
    case PROSES_DRAIN:
      prosesDrain();
      break;
  }
}

//---Private---