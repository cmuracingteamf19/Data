#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "CAN_config.h"
#include "ESP32CAN.h"

// ------------------ CONFIG ------------------
#define WIFI_SSID       "CMUF19"
#define WIFI_PASSWORD   "ilovef19"
#define FIREBASE_HOST   "test-data-f19-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH   "HbMfY46ykieKqMFzv4Kdl7vyim5ByDn4aOBr88nm"

#define CAN_TX_PIN      GPIO_NUM_27
#define CAN_RX_PIN      GPIO_NUM_26
#define CAN_SPEED       CAN_SPEED_500KBPS

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
CAN_device_t CAN_cfg;
const int RX_QUEUE_SIZE = 10;

uint32_t lastTimestamp = 0;   // ⏱ เก็บ timestamp จาก CAN ID 0x7FF

// ------------------ FUNC ------------------
float hexToFloat(uint32_t hex) {
  float f;
  memcpy(&f, &hex, sizeof(float));
  return f;
}

void sendToFirebase1(String path, String key, float val) { Firebase.RTDB.setFloat(&fbdo, path + "/" + key, val); }
void sendToFirebase1(String path, String key, int val)   { Firebase.RTDB.setInt(&fbdo, path + "/" + key, val); }
void sendToFirebase1(String path, String key, bool val)  { Firebase.RTDB.setBool(&fbdo, path + "/" + key, val); }
void sendToFirebase1(String path, String key, String val){ Firebase.RTDB.setString(&fbdo, path + "/" + key, val); }

void sendToFirebase2(String path, String key, float val) { Firebase.RTDB.setFloat(&fbdo, path + "/" + key, val); }
void sendToFirebase2(String path, String key, int val)   { Firebase.RTDB.setInt(&fbdo, path + "/" + key, val); }
void sendToFirebase2(String path, String key, bool val)  { Firebase.RTDB.setBool(&fbdo, path + "/" + key, val); }
void sendToFirebase2(String path, String key, String val){ Firebase.RTDB.setString(&fbdo, path + "/" + key, val); }

// ------------------ SETUP ------------------
void setup() {
  Serial.begin(115200);
  Serial.println("\n== ESP32: CAN + Firebase ==");

  // CAN setup
  CAN_cfg = { CAN_SPEED, CAN_TX_PIN, CAN_RX_PIN, xQueueCreate(RX_QUEUE_SIZE, sizeof(CAN_frame_t)) };
  ESP32Can.CANInit();
  Serial.println("CAN Bus ready @ 500 kbps");

  // WiFi setup
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) { Serial.print("."); delay(300); }
  Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());

  // Firebase setup
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.println("Firebase connected!\n");
}

// ------------------ LOOP ------------------
void loop() {
  CAN_frame_t frame;
  String path1 = "/ESP32_Data/Location1";
  String path2 = "/ESP32_Data/Location2";

  if (xQueueReceive(CAN_cfg.rx_queue, &frame, 3 / portTICK_PERIOD_MS) == pdTRUE) {
    uint8_t *d = frame.data.u8;

    // ✅ Timestamp frame จาก Bridge (ID 0x7FF)
    if (frame.MsgID == 0x7FF && frame.FIR.B.DLC >= 4) {
      lastTimestamp = ((uint32_t)d[3] << 24) | ((uint32_t)d[2] << 16) | ((uint32_t)d[1] << 8) | d[0];
      Serial.printf("⏱ Received Timestamp from CAN: %lu ms\n", lastTimestamp);
      return;
    }

    switch (frame.MsgID) {
      // ✅ Real-time Power
      case 0x09A10003: {
        if (frame.FIR.B.DLC == 8) {
          float current = hexToFloat(((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) | ((uint32_t)d[2] << 8) | d[3]);
          float voltage = hexToFloat(((uint32_t)d[4] << 24) | ((uint32_t)d[5] << 16) | ((uint32_t)d[6] << 8) | d[7]);
          Serial.printf("Real-Time >> I: %.2f A, V: %.2f V\n", current, voltage);
          sendToFirebase1(path1, "Current", current);
          sendToFirebase1(path1, "Voltage", voltage);
        }
        break;
      }

      // ✅ Battery Status
      case 0x09A10004: {
        if (frame.FIR.B.DLC == 8) {
          uint16_t socStatus = ((uint16_t)d[0] << 8) | d[1];
          float soc = (socStatus & 0x03FF) / 10.0;
          float cellHigh = (((uint16_t)d[2] << 8) | d[3]) / 1000.0;
          float cellLow  = (((uint16_t)d[4] << 8) | d[5]) / 1000.0;
          int8_t tHigh = (int8_t)d[6], tLow = (int8_t)d[7];
          Serial.printf("[BATTERY] SOC: %.1f%% | CH: %.3f V | CL: %.3f V | T:%d/%d °C\n", soc, cellHigh, cellLow, tHigh, tLow);
          sendToFirebase1(path1, "SOC_Percent", soc);
          sendToFirebase1(path1, "Cell_Highest_V", cellHigh);
          sendToFirebase1(path1, "Cell_Lowest_V", cellLow);
          sendToFirebase1(path1, "Temp_High", tHigh);
          sendToFirebase1(path1, "Temp_Low", tLow);
        }
        break;
      }

      // ✅ Controller Status
      case 0x09A10005: {
        if (frame.FIR.B.DLC == 8) {
          uint8_t fault = d[0], warning = d[1];
          bool charging = (d[5] >> 7) & 1;
          Serial.printf("[CTRL] Fault:%d | Warn:%d | Charging:%d\n", fault, warning, charging);
          sendToFirebase1(path1, "Fault_Code", fault);
          sendToFirebase1(path1, "Warning_Code", warning);
          sendToFirebase1(path1, "Charging", charging);
        }
        break;
      }

      // ✅ Motor/Inverter Data (0x7E1–0x7EA)
      case 0x7E1: { float udc = (d[0] | (d[1]<<8))*0.1; sendToFirebase2(path2, "Udc", udc); Serial.printf("0x7E1 → Udc: %.1f V\n", udc); break; }
      case 0x7E2: { float idc = (d[0] | (d[1]<<8))*0.1; sendToFirebase2(path2, "Idc", idc); Serial.printf("0x7E2 → Idc: %.1f A\n", idc); break; }
      case 0x7E3: { int16_t spd = d[0]|(d[1]<<8); sendToFirebase2(path2, "Speed", spd); Serial.printf("0x7E3 → Speed: %d rpm\n", spd); break; }
      case 0x7E4: { int16_t tm = d[0]|(d[1]<<8); sendToFirebase2(path2, "Motor_Temp", tm); Serial.printf("0x7E4 → Motor T: %d °C\n", tm); break; }
      case 0x7E5: { int16_t hs = d[0]|(d[1]<<8); sendToFirebase2(path2, "Heatsink_Temp", hs); Serial.printf("0x7E5 → Heatsink T: %d °C\n", hs); break; }
      case 0x7E6: { int16_t e = d[0]|(d[1]<<8); sendToFirebase2(path2, "Last_Error", e); break; }
      case 0x7E7: { float ge = (d[0]|(d[1]<<8))*0.1; sendToFirebase2(path2, "General_Error", ge); break; }
      case 0x7E8: { float ux = (d[0]|(d[1]<<8))*0.1; sendToFirebase2(path2, "Uaux", ux); break; }
      case 0x7E9: { int16_t dir = d[0]|(d[1]<<8); sendToFirebase2(path2, "Direction", dir); break; }
      case 0x7EA: { int16_t op = d[0]|(d[1]<<8); sendToFirebase2(path2, "Operation_Mode", op); break; }

      default:
        Serial.printf("Unknown ID: 0x%08X\n", frame.MsgID);
    }

    // ✅ ส่ง timestamp จากเฟรม 0x7FF ล่าสุดขึ้น Firebase
    if (lastTimestamp > 0) {
      String ts = String(lastTimestamp / 1000);
      sendToFirebase1(path1, "Timestamp1", ts);
      sendToFirebase2(path2, "Timestamp2",String(millis()/1000));
    }
  }
}
