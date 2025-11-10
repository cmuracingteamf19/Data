#include <SPI.h>
#include <mcp2515.h>
#include "driver/twai.h"

// ------------------- PIN CONFIG -------------------
#define CS_PIN   5      // CS ของ MCP2515
#define INT_PIN  4      // INT ของ MCP2515
#define CAN_TX   GPIO_NUM_27   // TX ของ SN65HVD230
#define CAN_RX   GPIO_NUM_26   // RX ของ SN65HVD230

// ------------------- OBJECTS -------------------
MCP2515 can250(CS_PIN);

// ------------------- CONFIG -------------------
#define TIMESTAMP_CAN_ID  0x7FF    // CAN ID ใหม่ที่ใช้ส่ง timestamp
#define TIMESTAMP_INTERVAL 100     // หน่วย ms (ความถี่การส่ง timestamp)

// ------------------- SETUP -------------------
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== CAN Bridge: MCP2515 (250 kbps) → TWAI (500 kbps) + Timestamp ===");

  // --- MCP2515 250 kbps ---
  SPI.begin();
  can250.reset();
  can250.setBitrate(CAN_250KBPS, MCP_8MHZ);   // ถ้าใช้ 16 MHz ให้เปลี่ยนเป็น MCP_16MHZ
  can250.setNormalMode();
  pinMode(INT_PIN, INPUT);
  Serial.println("MCP2515 started @250 kbps");

  // --- TWAI 500 kbps ---
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
    Serial.println("TWAI started @500 kbps");
  } else {
    Serial.println("❌ TWAI init failed!");
    while (1);
  }

  Serial.println("Bridge Ready!\n");
}

// ------------------- LOOP -------------------
void loop() {
  struct can_frame rxFrame;
  static unsigned long lastTimestamp = 0;

  // รับเฟรมจาก MCP2515 (250 kbps)
  if (can250.readMessage(&rxFrame) == MCP2515::ERROR_OK) {
    Serial.printf("[250→500] ID:0x%08lX DLC:%d Data:", (unsigned long)rxFrame.can_id, rxFrame.can_dlc);
    for (int i = 0; i < rxFrame.can_dlc; i++) Serial.printf(" %02X", rxFrame.data[i]);
    Serial.println();

    // เตรียมเฟรมใหม่สำหรับส่งต่อทาง TWAI (500 kbps)
    twai_message_t txMsg = {};

    // ตรวจสอบว่าเป็น extended หรือ standard frame
    if (rxFrame.can_id & CAN_EFF_FLAG) {
      txMsg.identifier = rxFrame.can_id & CAN_EFF_MASK;   // 29-bit ID
      txMsg.extd = 1;
    } else {
      txMsg.identifier = rxFrame.can_id & CAN_SFF_MASK;   // 11-bit ID
      txMsg.extd = 0;
    }

    txMsg.data_length_code = rxFrame.can_dlc;
    memcpy(txMsg.data, rxFrame.data, rxFrame.can_dlc);

    // ส่งออกทาง TWAI (500 kbps)
    esp_err_t result = twai_transmit(&txMsg, pdMS_TO_TICKS(5));
    if (result == ESP_OK)
      Serial.println("→ Sent to 500 kbps bus ✅");
    else
      Serial.println("⚠️ Send failed!");
  }

  // --- ส่ง Timestamp Frame ทุก ๆ TIMESTAMP_INTERVAL ms ---
  if (millis() - lastTimestamp >= TIMESTAMP_INTERVAL) {
    lastTimestamp = millis();

    twai_message_t tsMsg = {};
    tsMsg.identifier = TIMESTAMP_CAN_ID;  // ID พิเศษสำหรับ timestamp
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
