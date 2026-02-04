#include <SPI.h>
#include <mcp_can.h>
#include <driver/twai.h>

/* ================= MCP2515 ================= */
#define MCP_CS   5
#define MCP_INT  4
MCP_CAN CAN(MCP_CS);

/* ================= TJA1051 (TWAI) ================= */
#define TWAI_TX  27
#define TWAI_RX  26

/* ================= IO ================= */
#define INPUT_PIN    25    // อ่านค่า HVA (input)
#define PIN_EDGE320  14    // ติ๊ก HIGH 20ms จาก CAN ID 0x320
#define PIN_DIG312   13    // จาก ID 0x312 (>20)
#define PWM_312      15
#define PWM_313      22
#define PWM_330      32

/* ================= STATE ================= */
bool sent310 = false;
uint8_t last320 = 0;

/* ============= สำหรับ tick ขา 14 ============= */
bool tickActive = false;
unsigned long tickStart = 0;
const unsigned long TICK_DURATION = 20;  // 20ms

/* ================================================= */
/*        SEND CAN ID 0x310 (ONE-SHOT / HIGH)        */
/* ================================================= */
void handleTx310() {
  int hva = digitalRead(INPUT_PIN);
  Serial.print("HVA: ");
  Serial.println(hva);

  byte buf[1] = {hva};
  CAN.sendMsgBuf(0x310, 0, 1, buf);
}

/* ================================================= */
/*              RX CAN ID 0x320 (EDGE → TICK)        */
/* ================================================= */
void handleRx320(uint8_t value) {
  Serial.print("[RX] ID 0x320 = ");
  Serial.println(value);

  // ตรวจจับ rising edge
  if (value == 1 && last320 == 0) {
    Serial.println("     EDGE DETECTED -> TICK PIN14 HIGH (20ms)");
    digitalWrite(PIN_EDGE320, HIGH);
    tickActive = true;
    tickStart = millis();
  }

  if (value == 0 && last320 == 1) {
    Serial.println("     RESET -> VALUE RETURN TO 0");
  }

  last320 = value;
}

/* ================================================= */
/*                 RX CAN ID 0x312                   */
/* ================================================= */
void handleRx312(uint8_t value) {
  Serial.print("[RX] ID 0x312 = ");
  Serial.println(value);

  if (value > 20) {
    digitalWrite(PIN_DIG312, HIGH);
  } else {
    digitalWrite(PIN_DIG312, LOW);
  }

  if (value > 200) {
    digitalWrite(PWM_312, HIGH);
  } else {
    digitalWrite(PWM_312, LOW);
  }

}

/* ================================================= */
/*                 RX CAN ID 0x313                   */
/* ================================================= */
void handleRx313(uint8_t value) {
  Serial.print("[RX] ID 0x313 = ");
  Serial.println(value);
  if (value > 200) {
    digitalWrite(PWM_313, HIGH);

  } else {
    digitalWrite(PWM_313, LOW);
  }
}

/* ================================================= */
/*      RX CAN ID 0x330 (Throttle 2 bytes)           */
/* ================================================= */
void handleRx330(uint8_t t1, uint8_t t2) {
  Serial.print("[RX] ID 0x330 T1=");
  Serial.print(t1);
  Serial.print(" T2=");
  Serial.println(t2);
  analogWrite(PWM_330, t1);
}

/* ================================================= */
/*                 MCP2515 RX (FAST)                 */
/* ================================================= */
void handleMcpRx() {
  while (!digitalRead(MCP_INT)) {
    long unsigned int id;
    unsigned char len;
    unsigned char buf[8];

    CAN.readMsgBuf(&id, &len, buf);
    if (len < 1) continue;

    switch (id) {
      case 0x312: handleRx312(buf[0]); break;
      case 0x313: handleRx313(buf[0]); break;
      case 0x320: handleRx320(buf[0]); break;
      case 0x330:
        if (len >= 2) handleRx330(buf[0], buf[1]);
        break;
      default:
        break;
    }
  }
}

/* ================================================= */
/*      TJA1051 RX -> MCP TX (0x7EA -> 0x311)        */
/* ================================================= */
void handleTwaiRx() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    if (msg.identifier == 0x7EA && msg.data_length_code >= 1) {
      byte txBuf[1] = { msg.data[0] };
      CAN.sendMsgBuf(0x311, 0, 1, txBuf);
      Serial.print("[TWAI→MCP] 7EA->311 : ");
      Serial.println(msg.data[0]);
    }
  }
}

/* ================================================= */
/*                     SETUP                         */
/* ================================================= */
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 CAN RECEIVER (FAST MODE) START ===");

  pinMode(INPUT_PIN, INPUT);     // HVA input
  pinMode(PIN_EDGE320, OUTPUT);  // tick output
  pinMode(PIN_DIG312, OUTPUT);
  pinMode(MCP_INT, INPUT);

  digitalWrite(PIN_EDGE320, LOW);
  digitalWrite(PIN_DIG312, LOW);

  // เริ่มต้น MCP2515
  SPI.begin();
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN.setMode(MCP_NORMAL);
    Serial.println("MCP2515 READY");
  } else {
    Serial.println("[ERROR] MCP2515 INIT FAILED");
  }

  // เริ่มต้น TWAI (TJA1051)
  twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)TWAI_TX,
      (gpio_num_t)TWAI_RX,
      TWAI_MODE_NORMAL
    );
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    twai_start();
    Serial.println("TWAI READY");
  } else {
    Serial.println("[ERROR] TWAI INIT FAILED");
  }
}

/* ================================================= */
/*                      LOOP                         */
/* ================================================= */
unsigned long lastTx310 = 0;               // ตัวแปรจับเวลา
const unsigned long TX310_INTERVAL = 100;  // ส่งทุก 100ms

void loop() {
  handleTwaiRx();   // รับจาก TJA1051 → ส่งต่อ MCP
  handleMcpRx();    // อ่าน MCP2515 ต่อเนื่อง

  // ส่ง CAN ID 0x310 ทุกๆ 100ms
  if (millis() - lastTx310 >= TX310_INTERVAL) {
    handleTx310();
    lastTx310 = millis();
  }

  // ปิด PIN14 หลัง 20ms (non-blocking)
  if (tickActive && (millis() - tickStart >= TICK_DURATION)) {
    digitalWrite(PIN_EDGE320, LOW);
    tickActive = false;
    Serial.println("     TICK DONE -> PIN14 LOW (non-blocking)");
  }
}
