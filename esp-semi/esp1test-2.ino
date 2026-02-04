#include <SPI.h>
#include <mcp2515.h>
#include "driver/twai.h"

// ------------------- MOTOR A CONFIG -------------------
const int RPWM_A = 4;
const int LPWM_A = 2;

// ------------------- MOTOR B CONFIG -------------------
const int RPWM_B = 25;
const int LPWM_B = 33;

// ------------------- PWM CONFIG -------------------
const int freq = 20000;
const int resolution = 8;
int pwmSpeed_A = 255;   // ✅ ความเร็ว Motor A (0–255)
int pwmSpeed_B = 180;
;   // ✅ ความเร็ว Motor B (0–255)

// ------------------- CAN CONFIG -------------------
#define CS_PIN   5
#define INT_PIN  15
#define CAN_TX   GPIO_NUM_27
#define CAN_RX   GPIO_NUM_26

MCP2515 can250(CS_PIN);

#define TIMESTAMP_CAN_ID   0x7FF
#define TIMESTAMP_INTERVAL 100
unsigned long lastTimestamp = 0;

// ------------------- SETUP -------------------
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== CAN Bridge + Dual BTS7960 Motor Control (Independent Speed) ===");

  // --- Motor A setup ---
  pinMode(RPWM_A, OUTPUT);
  pinMode(LPWM_A, OUTPUT);
  ledcSetup(0, freq, resolution);
  ledcSetup(1, freq, resolution);
  ledcAttachPin(RPWM_A, 0);
  ledcAttachPin(LPWM_A, 1);
  Serial.println("✅ Motor A setup complete");

  // --- Motor B setup ---
  pinMode(RPWM_B, OUTPUT);
  pinMode(LPWM_B, OUTPUT);
  ledcSetup(2, freq, resolution);
  ledcSetup(3, freq, resolution);
  ledcAttachPin(RPWM_B, 2);
  ledcAttachPin(LPWM_B, 3);
  Serial.println("✅ Motor B setup complete");

  // --- MCP2515 setup ---
  SPI.begin();
  can250.reset();
  can250.setBitrate(CAN_250KBPS, MCP_8MHZ);
  can250.setNormalMode();
  pinMode(INT_PIN, INPUT);
  Serial.println("✅ MCP2515 started @250 kbps");

  // --- TWAI setup ---
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
    Serial.println("✅ TWAI started @500 kbps");
  } else {
    Serial.println("❌ TWAI init failed!");
    while (1);
  }

  Serial.println("Bridge + Dual Motor Ready!\n");
}

// ------------------- LOOP -------------------
void loop() {
  handleCANBridge();
  handleTimestamp();
  maintainMotors();
}

// ------------------- FUNCTIONS -------------------
void handleCANBridge() {
  struct can_frame rxFrame;

  if (can250.readMessage(&rxFrame) == MCP2515::ERROR_OK) {
    Serial.printf("[250→500] ID:0x%08lX DLC:%d Data:", (unsigned long)rxFrame.can_id, rxFrame.can_dlc);
    for (int i = 0; i < rxFrame.can_dlc; i++) Serial.printf(" %02X", rxFrame.data[i]);
    Serial.println();

    twai_message_t txMsg = {};
    if (rxFrame.can_id & CAN_EFF_FLAG) {
      txMsg.identifier = rxFrame.can_id & CAN_EFF_MASK;
      txMsg.extd = 1;
    } else {
      txMsg.identifier = rxFrame.can_id & CAN_SFF_MASK;
      txMsg.extd = 0;
    }

    txMsg.data_length_code = rxFrame.can_dlc;
    memcpy(txMsg.data, rxFrame.data, rxFrame.can_dlc);

    esp_err_t result = twai_transmit(&txMsg, pdMS_TO_TICKS(5));
    if (result == ESP_OK)
      Serial.println("→ Sent to 500 kbps bus ✅");
    else
      Serial.println("⚠️ Send failed!");
  }
}

// ส่ง timestamp frame ทุก TIMESTAMP_INTERVAL ms
void handleTimestamp() {
  if (millis() - lastTimestamp >= TIMESTAMP_INTERVAL) {
    lastTimestamp = millis();

    twai_message_t tsMsg = {};
    tsMsg.identifier = TIMESTAMP_CAN_ID;
    tsMsg.extd = 0;
    tsMsg.data_length_code = 4;

    uint32_t t = millis();
    tsMsg.data[0] = (t & 0xFF);
    tsMsg.data[1] = (t >> 8) & 0xFF;
    tsMsg.data[2] = (t >> 16) & 0xFF;
    tsMsg.data[3] = (t >> 24) & 0xFF;

    if (twai_transmit(&tsMsg, pdMS_TO_TICKS(5)) == ESP_OK)
      Serial.printf("⏱ Timestamp sent [ID:0x%03X] = %lu ms\n", TIMESTAMP_CAN_ID, t);
  }
}

// ควบคุมมอเตอร์ทั้งสอง (เดินหน้า, ความเร็วแยกกัน)
void maintainMotors() {
  // Motor A
  ledcWrite(0, pwmSpeed_A);  // RPWM เดินหน้า
  ledcWrite(1, 0);           // LPWM ปิด

  // Motor B
  ledcWrite(2, pwmSpeed_B);  // RPWM เดินหน้า
  ledcWrite(3, 0);           // LPWM ปิด
}
