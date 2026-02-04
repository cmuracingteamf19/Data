#include <Arduino.h>
#include "driver/twai.h"

/* ===================== PIN ===================== */
#define CAN_TX_PIN     GPIO_NUM_27
#define CAN_RX_PIN     GPIO_NUM_26

#define BRAKE1_PIN     34
#define BRAKE2_PIN     35
#define APPS1_PIN      36
#define APPS2_PIN      39

#define RTD_BUTTON_PIN 32
#define BUZZER_PIN     12

/* ===================== CAN ID ===================== */
#define CAN_ID_HVA       0x310
#define CAN_ID_OPMODE    0x311
#define CAN_ID_BRAKE1    0x312
#define CAN_ID_BRAKE2    0x313
#define CAN_ID_RTD       0x320
#define CAN_ID_THROTTLE  0x330

/* ===================== MAP THRESHOLD ===================== */
#define APPS_DIFF_MAP       25
#define BRAKE_THROTTLE_MAP  64
#define APPS_RELEASE_MAP    13
#define BRAKE_ACTIVE_MAP    10
#define APPS_CORR_TIME_MS   1000
#define BRAKE_RELEASE_MAP   0

/* ===================== SENSOR CALIBRATION ===================== */
#define BRK1_RAW_MIN   0
#define BRK1_RAW_MAX   4095
#define BRK2_RAW_MIN   0
#define BRK2_RAW_MAX   1000
#define APPS1_RAW_MIN  560
#define APPS1_RAW_MAX  3000
#define APPS2_RAW_MIN  560
#define APPS2_RAW_MAX  3000

#define MAP_OUT_MIN    0
#define MAP_OUT_MAX    255

/* ===================== STATE ===================== */
enum SystemState {
  STATE_IDLE,
  STATE_RTD_ACTIVE,
  STATE_DRIVE,
  STATE_FAULT
};

SystemState state = STATE_IDLE;

/* ===================== GLOBAL ===================== */
bool hva = false;
bool hva_prev = false; // สำหรับตรวจจับ HV หาย
uint8_t opmode = 0;
bool rtd_btn = false;
bool rtd_sent_once = false;

/* ---------- BUZZER ---------- */
bool buzzer_active = false;
bool buzzer_done   = false;
uint8_t buzzer_step = 0;
uint32_t buzzer_step_start_ms = 0;

/* ---------- RAW ---------- */
uint16_t brake1_raw, brake2_raw;
uint16_t apps1_raw, apps2_raw;

/* ---------- MAP ---------- */
uint8_t brake1_map, brake2_map;
uint8_t apps1_map,  apps2_map;

/* ---------- FLAGS ---------- */
bool apps_fault = false;
bool brake_throttle_cut = false;
uint32_t apps_timer = 0;

/* ---------- SERIAL ---------- */
uint32_t lastPrintMs = 0;
#define PRINT_INTERVAL_MS 100

/* ===================== BUZZER PATTERN ===================== */
struct BuzzerStep {
  bool level;
  uint16_t duration_ms;
};

const BuzzerStep buzzer_pattern[] = {
  {HIGH,100}, {LOW,100},
  {HIGH,1300},{LOW,100},
  {HIGH,100}, {LOW,100},
  {HIGH,100}, {LOW,100},
  {HIGH,100}, {LOW,0}
};

/* ===================== BUZZER UPDATE ===================== */
void updateBuzzer() {
  if (!buzzer_active) return;

  uint32_t now = millis();

  if (buzzer_step_start_ms == 0) {
    buzzer_step_start_ms = now;
    digitalWrite(BUZZER_PIN, buzzer_pattern[buzzer_step].level);
  }

  if (now - buzzer_step_start_ms >= buzzer_pattern[buzzer_step].duration_ms) {
    buzzer_step++;
    buzzer_step_start_ms = now;

    if (buzzer_pattern[buzzer_step].duration_ms == 0) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzer_active = false;
      buzzer_done = true;
      buzzer_step = 0;
      buzzer_step_start_ms = 0;
      return;
    }
    digitalWrite(BUZZER_PIN, buzzer_pattern[buzzer_step].level);
  }
}

/* ===================== MAP FUNCTIONS ===================== */
uint8_t mapBrake1(uint16_t raw) {
  raw = constrain(raw, BRK1_RAW_MIN, BRK1_RAW_MAX);
  return map(raw, BRK1_RAW_MIN, BRK1_RAW_MAX, MAP_OUT_MIN, MAP_OUT_MAX);
}
uint8_t mapBrake2(uint16_t raw) {
  raw = constrain(raw, BRK2_RAW_MIN, BRK2_RAW_MAX);
  return map(raw, BRK2_RAW_MIN, BRK2_RAW_MAX, MAP_OUT_MIN, MAP_OUT_MAX);
}
uint8_t mapApps1(uint16_t raw) {
  raw = constrain(raw, APPS1_RAW_MIN, APPS1_RAW_MAX);
  return map(raw, APPS1_RAW_MIN, APPS1_RAW_MAX, MAP_OUT_MIN, MAP_OUT_MAX);
}
uint8_t mapApps2(uint16_t raw) {
  raw = constrain(raw, APPS2_RAW_MIN, APPS2_RAW_MAX);
  return map(raw, APPS2_RAW_MIN, APPS2_RAW_MAX, MAP_OUT_MIN, MAP_OUT_MAX);
}

/* ===================== CAN ===================== */
void setupCAN() {
  twai_general_config_t g =
    TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g, &t, &f);
  twai_start();
}

void CAN_Rx_Task(void *pv) {
  twai_message_t rx;
  while (1) {
    if (twai_receive(&rx, portMAX_DELAY) == ESP_OK) {
      if (rx.identifier == CAN_ID_HVA) hva = rx.data[0];
      else if (rx.identifier == CAN_ID_OPMODE) opmode = rx.data[0];
    }
  }
}

/* ===================== APPS CHECK ===================== */
void updateAppsPlausibility() {
  uint8_t diff = abs((int)apps1_map - (int)apps2_map);

  if (diff > APPS_DIFF_MAP) {
    if (!apps_timer) apps_timer = millis();
    if (millis() - apps_timer > APPS_CORR_TIME_MS)
      apps_fault = true;
  } else {
    apps_timer = 0;
  }

  if (brake1_map > BRAKE_ACTIVE_MAP &&
      ((apps1_map + apps2_map) / 2) > BRAKE_THROTTLE_MAP) {
    brake_throttle_cut = true;
    apps_fault = true;
  } else {
    brake_throttle_cut = false;
  }

  if (apps_fault &&
      apps1_map  < APPS_RELEASE_MAP &&
      apps2_map  < APPS_RELEASE_MAP) {
    apps_fault = false;
  }
}

/* ===================== STATE ===================== */
void updateState() {
  if (!hva && (state == STATE_DRIVE || state == STATE_RTD_ACTIVE)) {
    state = STATE_IDLE;
    rtd_sent_once = false;
    buzzer_done = false;
    buzzer_active = false;
    buzzer_step = 0;
    buzzer_step_start_ms = 0;
    return;
  }

  if (!hva) {
    state = STATE_IDLE;
    rtd_sent_once = false;
    buzzer_done = false;
    return;
  }

  switch (state) {
    case STATE_IDLE:
      if (rtd_btn && brake1_map > BRAKE_ACTIVE_MAP) {
        state = STATE_RTD_ACTIVE;
        buzzer_active = true;
        buzzer_done = false;
      }
      break;

    case STATE_RTD_ACTIVE:
      if (opmode == 1) state = STATE_DRIVE;
      break;

    case STATE_DRIVE:
      if (apps_fault) state = STATE_FAULT;
      break;

    case STATE_FAULT:
      if (!apps_fault) state = STATE_DRIVE;
      break;
  }
}

/* ===================== CAN TX ===================== */
void CAN_Tx_Task(void *pv) {
  twai_message_t tx = {};
  tx.data_length_code = 1;

  while (1) {
    tx.identifier = CAN_ID_BRAKE1;
    tx.data[0] = brake1_map;
    twai_transmit(&tx, pdMS_TO_TICKS(5));

    tx.identifier = CAN_ID_BRAKE2;
    tx.data[0] = brake2_map;
    twai_transmit(&tx, pdMS_TO_TICKS(5));

    // หาก HV หาย ให้ส่ง Throttle = 0 หนึ่งครั้ง
    if (!hva && hva_prev) {
      tx.identifier = CAN_ID_THROTTLE;
      tx.data_length_code = 2;
      tx.data[0] = 0;
      tx.data[1] = 0;
      twai_transmit(&tx, pdMS_TO_TICKS(5));
      tx.data_length_code = 1;
    }

    if (state == STATE_RTD_ACTIVE && buzzer_done && !rtd_sent_once) {
      tx.identifier = CAN_ID_RTD;
      tx.data[0] = 1;
      for (int i = 0; i < 5; i++) {
        twai_transmit(&tx, pdMS_TO_TICKS(5));
      }
      rtd_sent_once = true;
    }

    if (state == STATE_DRIVE || state == STATE_FAULT) {
      tx.identifier = CAN_ID_THROTTLE;
      tx.data_length_code = 2;
      if (state == STATE_DRIVE && !apps_fault) {
        tx.data[0] = apps1_map;
        tx.data[1] = apps2_map;
      } else {
        tx.data[0] = 0;
        tx.data[1] = 0;
      }
      twai_transmit(&tx, pdMS_TO_TICKS(5));
      tx.data_length_code = 1;
    }

    hva_prev = hva;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);
  pinMode(RTD_BUTTON_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  setupCAN();
  xTaskCreate(CAN_Rx_Task, "CAN_RX", 4096, NULL, 3, NULL);
  xTaskCreate(CAN_Tx_Task, "CAN_TX", 4096, NULL, 2, NULL);
}

/* ===================== LOOP ===================== */
void loop() {
  rtd_btn = digitalRead(RTD_BUTTON_PIN);

  brake1_raw = analogRead(BRAKE1_PIN);
  brake2_raw = analogRead(BRAKE2_PIN);
  apps1_raw  = analogRead(APPS1_PIN);
  apps2_raw  = analogRead(APPS2_PIN);

  brake1_map = mapBrake1(brake1_raw);
  brake2_map = mapBrake2(brake2_raw);
  apps1_map  = mapApps1(apps1_raw);
  apps2_map  = mapApps2(apps2_raw);

  if (apps1_raw < 200 || apps2_raw < 200) {
    apps1_map = 0;
    apps2_map = 0;
  }

  updateAppsPlausibility();
  updateState();
  updateBuzzer();

  if (millis() - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = millis();

    Serial.print("RAW | B1:");
    Serial.print(brake1_raw);
    Serial.print(" B2:");
    Serial.print(brake2_raw);
    Serial.print(" A1:");
    Serial.print(apps1_raw);
    Serial.print(" A2:");
    Serial.print(apps2_raw);

    Serial.print(" || MAP | B1:");
    Serial.print(brake1_map);
    Serial.print(" B2:");
    Serial.print(brake2_map);
    Serial.print(" A1:");
    Serial.print(apps1_map);
    Serial.print(" A2:");
    Serial.print(apps2_map);

    Serial.print(" || STATE:");
    Serial.print(state);
    Serial.print(" HVA:");
    Serial.print(hva);
    Serial.print(" OPM:");
    Serial.print(opmode);
    Serial.print(" RTD_BTN:");
    Serial.print(rtd_btn);
    Serial.print(" RTD_SENT:");
    Serial.print(rtd_sent_once);

    Serial.print(" BUZ:");
    Serial.print(buzzer_active);
    Serial.print(" BUZ_DONE:");
    Serial.print(buzzer_done);

    Serial.print(" CUT:");
    Serial.print(brake_throttle_cut);

    Serial.print(" FAULT:");
    Serial.println(apps_fault);
  }
}
