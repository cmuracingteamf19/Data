#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "driver/twai.h"

// ------------------ CONFIG ------------------
#define WIFI_SSID       "CMUF19"
#define WIFI_PASSWORD   "ilovef19"
#define FIREBASE_HOST   "test-data-f19-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH   "HbMfY46ykieKqMFzv4Kdl7vyim5ByDn4aOBr88nm"

#define CAN_TX_PIN      GPIO_NUM_27
#define CAN_RX_PIN      GPIO_NUM_26

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

uint32_t lastTimestamp = 0;
uint32_t lastSendTime = 0;
const uint32_t SEND_INTERVAL = 1000;  // ส่ง Firebase ทุก 1 วิ

// เก็บค่าล่าสุดที่จะส่ง (ตัวแปรเหมือนเดิมทุกตัว)
float currentVal = 0, voltageVal = 0;
float socVal = 0, cellHighVal = 0, cellLowVal = 0;
int8_t tempHighVal = 0, tempLowVal = 0;
uint8_t faultVal = 0, warningVal = 0; bool chargingVal = 0;
float udcVal = 0, idcVal = 0, genErrVal = 0, uauxVal = 0;
int16_t speedVal = 0, motorTempVal = 0, hsTempVal = 0, lastErrVal = 0, dirVal = 0, opVal = 0;
String ts1 = "0", ts2 = "0";

// ------------------ FUNC ------------------
float hexToFloat(uint32_t hex) {
  float f;
  memcpy(&f, &hex, sizeof(float));
  return f;
}

// ------------------ SETUP ------------------
void setup() {
  Serial.begin(115200);
  Serial.println("\n== ESP32: Optimized TWAI + Firebase (Same Keys) ==");

  // ✅ TWAI setup
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK)
    Serial.println("TWAI started @ 500 kbps");
  else {
    Serial.println("❌ Failed to start TWAI");
    while (true) delay(1000);
  }

  // ✅ WiFi setup
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) { Serial.print("."); delay(300); }
  Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());

  // ✅ Firebase setup
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.println("Firebase ready!\n");
}

// ------------------ LOOP ------------------
void loop() {
  twai_message_t frame;
  String path1 = "/ESP32_Data/Location1";
  String path2 = "/ESP32_Data/Location2";

  // ✅ Non-blocking receive
  if (twai_receive(&frame, 0) == ESP_OK) {
    uint8_t *d = frame.data;

    // Timestamp frame
    if (frame.identifier == 0x7FF && frame.data_length_code >= 4) {
      lastTimestamp = ((uint32_t)d[3] << 24) | ((uint32_t)d[2] << 16) | ((uint32_t)d[1] << 8) | d[0];
      Serial.printf("⏱ Timestamp: %lu ms\n", lastTimestamp);
      return;
    }

    switch (frame.identifier) {
      case 0x09A10003: {  // Real-time Power
        if (frame.data_length_code == 8) {
          currentVal = hexToFloat(((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) | ((uint32_t)d[2] << 8) | d[3]);
          voltageVal = hexToFloat(((uint32_t)d[4] << 24) | ((uint32_t)d[5] << 16) | ((uint32_t)d[6] << 8) | d[7]);
        }
        break;
      }

      case 0x09A10004: {  // Battery
        if (frame.data_length_code == 8) {
          uint16_t socStatus = ((uint16_t)d[0] << 8) | d[1];
          socVal = (socStatus & 0x03FF) / 10.0;
          cellHighVal = (((uint16_t)d[2] << 8) | d[3]) / 1000.0;
          cellLowVal  = (((uint16_t)d[4] << 8) | d[5]) / 1000.0;
          tempHighVal = (int8_t)d[6]; tempLowVal = (int8_t)d[7];
        }
        break;
      }

      case 0x09A10005: {  // Controller
        if (frame.data_length_code == 8) {
          faultVal = d[0]; warningVal = d[1];
          chargingVal = (d[5] >> 7) & 1;
        }
        break;
      }

      // Motor / Inverter
      case 0x7E1: { udcVal = (d[0]|(d[1]<<8))*0.1; break; }
      case 0x7E2: { idcVal = (d[0]|(d[1]<<8))*0.1; break; }
      case 0x7E3: { speedVal = d[0]|(d[1]<<8); break; }
      case 0x7E4: { motorTempVal = d[0]|(d[1]<<8); break; }
      case 0x7E5: { hsTempVal = d[0]|(d[1]<<8); break; }
      case 0x7E6: { lastErrVal = d[0]|(d[1]<<8); break; }
      case 0x7E7: { genErrVal = (d[0]|(d[1]<<8))*0.1; break; }
      case 0x7E8: { uauxVal = (d[0]|(d[1]<<8))*0.1; break; }
      case 0x7E9: { dirVal = d[0]|(d[1]<<8); break; }
      case 0x7EA: { opVal = d[0]|(d[1]<<8); break; }

      default:
        break;
    }

    if (lastTimestamp > 0) {
      ts1 = String(lastTimestamp / 1000);
      ts2 = String(millis() / 1000);
    }
  }

  // ✅ ส่ง Firebase ทุก 1 วิ (ค่าเหมือนเดิมทุก key)
  if (millis() - lastSendTime >= SEND_INTERVAL) {
    Firebase.RTDB.setFloat(&fbdo, path1 + "/Current", currentVal);
    Firebase.RTDB.setFloat(&fbdo, path1 + "/Voltage", voltageVal);
    Firebase.RTDB.setFloat(&fbdo, path1 + "/SOC_Percent", socVal);
    Firebase.RTDB.setFloat(&fbdo, path1 + "/Cell_Highest_V", cellHighVal);
    Firebase.RTDB.setFloat(&fbdo, path1 + "/Cell_Lowest_V", cellLowVal);
    Firebase.RTDB.setInt(&fbdo,   path1 + "/Temp_High", tempHighVal);
    Firebase.RTDB.setInt(&fbdo,   path1 + "/Temp_Low", tempLowVal);
    Firebase.RTDB.setInt(&fbdo,   path1 + "/Fault_Code", faultVal);
    Firebase.RTDB.setInt(&fbdo,   path1 + "/Warning_Code", warningVal);
    Firebase.RTDB.setBool(&fbdo,  path1 + "/Charging", chargingVal);
    Firebase.RTDB.setString(&fbdo, path1 + "/Timestamp1", ts1);

    Firebase.RTDB.setFloat(&fbdo, path2 + "/Udc", udcVal);
    Firebase.RTDB.setFloat(&fbdo, path2 + "/Idc", idcVal);
    Firebase.RTDB.setInt(&fbdo,   path2 + "/Speed", speedVal);
    Firebase.RTDB.setInt(&fbdo,   path2 + "/Motor_Temp", motorTempVal);
    Firebase.RTDB.setInt(&fbdo,   path2 + "/Heatsink_Temp", hsTempVal);
    Firebase.RTDB.setInt(&fbdo,   path2 + "/Last_Error", lastErrVal);
    Firebase.RTDB.setFloat(&fbdo, path2 + "/General_Error", genErrVal);
    Firebase.RTDB.setFloat(&fbdo, path2 + "/Uaux", uauxVal);
    Firebase.RTDB.setInt(&fbdo,   path2 + "/Direction", dirVal);
    Firebase.RTDB.setInt(&fbdo,   path2 + "/Operation_Mode", opVal);
    Firebase.RTDB.setString(&fbdo, path2 + "/Timestamp2", ts2);

    lastSendTime = millis();
    Serial.println("✅ Firebase updated");
  }
}
