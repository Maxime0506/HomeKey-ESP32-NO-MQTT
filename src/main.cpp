#include <hkAuthContext.h>
#include <HomeKey.h>
#include <utils.h>
#include <HomeSpan.h>
#include <PN532_SPI.h>
#include <PN532.h>
#include <HAP.h>
#include <chrono>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <HK_HomeKit.h>
#include "config.h"
#include <esp_ota_ops.h>
#include <esp_task.h>
#include <pins_arduino.h>
#include <Adafruit_NeoPixel.h>
#define RELAY_PIN 27

const char* TAG = "MAIN";
int debug = 1; // Debug-Modus: 1 aktiviert, 0 deaktiviert
enum lockStates
{
  UNLOCKED,
  LOCKED,
  JAMMED,
  UNKNOWN,
  UNLOCKING,
  LOCKING
};

AsyncWebServer webServer(80);
PN532_SPI pn532spi(SS, SCK, MISO, MOSI);
PN532 nfc(pn532spi);
QueueHandle_t gpio_led_handle = nullptr;

nvs_handle savedData;
readerData_t readerData;
uint8_t ecpData[18] = { 0x6A, 0x2, 0xCB, 0x2, 0x6, 0x2, 0x11, 0x0 };
std::map<HK_COLOR, const char*> hk_color_vals = {{TAN, "AQTO1doA"}, {GOLD, "AQSq1uwA"}, {SILVER, "AQTj4+MA"}, {BLACK, "AQQAAAAA"}};

namespace espConfig
{
  struct misc_config_t
  {
    std::string deviceName = DEVICE_NAME;
    std::string otaPasswd = OTA_PWD;
    uint8_t hk_key_color = HOMEKEY_COLOR;
    std::string setupCode = SETUP_CODE;
    bool lockAlwaysUnlock = HOMEKEY_ALWAYS_UNLOCK;
    bool lockAlwaysLock = HOMEKEY_ALWAYS_LOCK;
    uint8_t controlPin = HS_PIN;
    uint8_t hsStatusPin = HS_STATUS_LED;
    uint8_t nfcNeopixelPin = NFC_NEOPIXEL_PIN;
    uint8_t nfcSuccessPin = NFC_SUCCESS_PIN;
    uint16_t nfcSuccessTime = NFC_SUCCESS_TIME;
    bool nfcSuccessHL = NFC_SUCCESS_HL;
    uint8_t nfcFailPin = NFC_FAIL_PIN;
    uint16_t nfcFailTime = NFC_FAIL_TIME;
    bool nfcFailHL = NFC_FAIL_HL;
    bool gpioActionEnable = GPIO_ACTION_ENABLE;
    uint8_t gpioActionPin = GPIO_ACTION_PIN;
    bool gpioActionLockState = GPIO_ACTION_LOCK_STATE;
    bool gpioActionUnlockState = GPIO_ACTION_UNLOCK_STATE;
  } miscConfig;
}
JSONCONS_ALL_MEMBER_TRAITS(espConfig::misc_config_t, deviceName, hk_key_color, lockAlwaysUnlock, lockAlwaysLock, controlPin, hsStatusPin, nfcSuccessPin, nfcNeopixelPin, nfcSuccessHL, nfcFailPin, nfcFailHL, gpioActionEnable, gpioActionPin, gpioActionLockState, gpioActionUnlockState, otaPasswd, setupCode)

KeyFlow hkFlow = KeyFlow::kFlowFAST;
SpanCharacteristic* lockCurrentState;
SpanCharacteristic* lockTargetState;

Adafruit_NeoPixel pixels(1, espConfig::miscConfig.nfcNeopixelPin, NEO_GRB + NEO_KHZ800);

bool save_to_nvs() {
  std::vector<uint8_t> cborBuf;
  jsoncons::msgpack::encode_msgpack(readerData, cborBuf);
  esp_err_t set_nvs = nvs_set_blob(savedData, "READERDATA", cborBuf.data(), cborBuf.size());
  esp_err_t commit_nvs = nvs_commit(savedData);
  LOG(D, "NVS SET STATUS: %s", esp_err_to_name(set_nvs));
  LOG(D, "NVS COMMIT STATUS: %s", esp_err_to_name(commit_nvs));
  return !set_nvs && !commit_nvs;
}

struct LockManagement : Service::LockManagement
{
  SpanCharacteristic* lockControlPoint;
  SpanCharacteristic* version;
  const char* TAG = "LockManagement";

  LockManagement() : Service::LockManagement() {

    LOG(D, "Configuring LockManagement"); // initialization message

    lockControlPoint = new Characteristic::LockControlPoint();
    version = new Characteristic::Version();

  } // end constructor

}; // end LockManagement

// Function to calculate CRC16
void crc16a(unsigned char* data, unsigned int size, unsigned char* result) {
  unsigned short w_crc = 0x6363;

  for (unsigned int i = 0; i < size; ++i) {
    unsigned char byte = data[i];
    byte = (byte ^ (w_crc & 0x00FF));
    byte = ((byte ^ (byte << 4)) & 0xFF);
    w_crc = ((w_crc >> 8) ^ (byte << 8) ^ (byte << 3) ^ (byte >> 4)) & 0xFFFF;
  }

  result[0] = static_cast<unsigned char>(w_crc & 0xFF);
  result[1] = static_cast<unsigned char>((w_crc >> 8) & 0xFF);
}

// Function to append CRC16 to data
void with_crc16(unsigned char* data, unsigned int size, unsigned char* result) {
  crc16a(data, size, result);
}

struct LockMechanism : Service::LockMechanism
{
  const char* TAG = "LockMechanism";

  LockMechanism() : Service::LockMechanism() {
    LOG(I, "Configuring LockMechanism"); // initialization message
    lockCurrentState = new Characteristic::LockCurrentState(1, true);
    lockTargetState = new Characteristic::LockTargetState(1, true);
    memcpy(ecpData + 8, readerData.reader_gid.data(), readerData.reader_gid.size());
    with_crc16(ecpData, 16, ecpData + 16);
    if (espConfig::miscConfig.gpioActionEnable && espConfig::miscConfig.gpioActionPin != 255) {
      if (lockCurrentState->getVal() == lockStates::LOCKED) {
        digitalWrite(espConfig::miscConfig.gpioActionPin, espConfig::miscConfig.gpioActionLockState);
      }
      else if (lockCurrentState->getVal() == lockStates::UNLOCKED) {
        digitalWrite(espConfig::miscConfig.gpioActionPin, espConfig::miscConfig.gpioActionUnlockState);
      }
    }
  } // end constructor

  boolean update() {
    int targetState = lockTargetState->getNewVal();
    LOG(I, "New LockState=%d, Current LockState=%d", targetState, lockCurrentState->getVal());
    if (espConfig::miscConfig.gpioActionEnable && espConfig::miscConfig.gpioActionPin != 255) {
      switch (targetState)
      {
        case lockStates::UNLOCKED:
          digitalWrite(espConfig::miscConfig.gpioActionPin, espConfig::miscConfig.gpioActionUnlockState);
          lockCurrentState->setVal(lockStates::UNLOCKED);
          break;

        case lockStates::LOCKED:
          digitalWrite(espConfig::miscConfig.gpioActionPin, espConfig::miscConfig.gpioActionLockState);
          lockCurrentState->setVal(lockStates::LOCKED);
          break;

        default:
          break;
      }
    }
    return (true);
  }
};

struct NFCAccess : Service::NFCAccess
{
  SpanCharacteristic* configurationState;
  SpanCharacteristic* nfcControlPoint;
  SpanCharacteristic* nfcSupportedConfiguration;
  const char* TAG = "NFCAccess";

  NFCAccess() : Service::NFCAccess() {
    LOG(I, "Configuring NFCAccess"); // initialization message
    configurationState = new Characteristic::ConfigurationState();
    nfcControlPoint = new Characteristic::NFCAccessControlPoint();
    TLV8 conf(NULL, 0);
    conf.add(0x01, 0x10);
    conf.add(0x02, 0x10);
    nfcSupportedConfiguration = new Characteristic::NFCAccessSupportedConfiguration(conf);
  }

  boolean update() {
    LOG(D, "PROVISIONED READER KEY: %s", utils::bufToHexString(readerData.reader_pk.data(), readerData.reader_pk.size()).c_str());
    LOG(D, "READER GROUP IDENTIFIER: %s", utils::bufToHexString(readerData.reader_gid.data(), readerData.reader_gid.size()).c_str());
    LOG(D, "READER UNIQUE IDENTIFIER: %s", utils::bufToHexString(readerData.reader_id.data(), readerData.reader_id.size()).c_str());

    TLV8 ctrlData(NULL, 0);
    nfcControlPoint->getNewTLV(ctrlData);
    std::vector<uint8_t> tlvData(ctrlData.pack_size());
    ctrlData.pack(tlvData.data());
    if (tlvData.size() == 0)
      return false;
    LOG(D, "Decoded data: %s", utils::bufToHexString(tlvData.data(), tlvData.size()).c_str());
    LOG(D, "Decoded data length: %d", tlvData.size());
    HK_HomeKit hkCtx(readerData, savedData, "READERDATA", tlvData);
    std::vector<uint8_t> result = hkCtx.processResult();
    if (readerData.reader_gid.size() > 0) {
      memcpy(ecpData + 8, readerData.reader_gid.data(), readerData.reader_gid.size());
      with_crc16(ecpData, 16, ecpData + 16);
    }
    TLV8 res(NULL, 0);
    res.unpack(result.data(), result.size());
    nfcControlPoint->setTLV(res, false);
    return true;
  }

};

void deleteReaderData(const char* buf) {
  esp_err_t erase_nvs = nvs_erase_key(savedData, "READERDATA");
  esp_err_t commit_nvs = nvs_commit(savedData);
  readerData.issuers.clear();
  readerData.reader_gid.clear();
  readerData.reader_id.clear();
  readerData.reader_pk.clear();
  readerData.reader_pk_x.clear();
  readerData.reader_sk.clear();
  LOG(D, "*** NVS W STATUS");
  LOG(D, "ERASE: %s", esp_err_to_name(erase_nvs));
  LOG(D, "COMMIT: %s", esp_err_to_name(commit_nvs));
  LOG(D, "*** NVS W STATUS");
}

void pairCallback() {
  if (HAPClient::nAdminControllers() == 0) {
    deleteReaderData(NULL);
    return;
  }
  for (auto it = homeSpan.controllerListBegin(); it != homeSpan.controllerListEnd(); ++it) {
    std::vector<uint8_t> id = utils::getHashIdentifier(it->getLTPK(), 32, true);
    LOG(D, "Found allocated controller - Hash: %s", utils::bufToHexString(id.data(), 8).c_str());
    hkIssuer_t* foundIssuer = nullptr;
    for (auto&& issuer : readerData.issuers) {
      if (std::equal(issuer.issuer_id.begin(), issuer.issuer_id.end(), id.begin())) {
        LOG(D, "Issuer %s already added, skipping", utils::bufToHexString(issuer.issuer_id.data(), issuer.issuer_id.size()).c_str());
        foundIssuer = &issuer;
        break;
      }
    }
    if (foundIssuer == nullptr) {
      LOG(D, "Adding new issuer - ID: %s", utils::bufToHexString(id.data(), 8).c_str());
      hkIssuer_t newIssuer;
      newIssuer.issuer_id = std::vector<uint8_t>{id.begin(), id.begin() + 8};
      newIssuer.issuer_pk.insert(newIssuer.issuer_pk.begin(), it->getLTPK(), it->getLTPK() + 32);
      readerData.issuers.emplace_back(newIssuer);
    }
  }
  save_to_nvs();
}

void setFlow(const char* buf) {
  switch (buf[1]) {
  case '0':
    hkFlow = KeyFlow::kFlowFAST;
    Serial.println("FAST Flow");
    break;

  case '1':
    hkFlow = KeyFlow::kFlowSTANDARD;
    Serial.println("STANDARD Flow");
    break;
  case '2':
    hkFlow = KeyFlow::kFlowATTESTATION;
    Serial.println("ATTESTATION Flow");
    break;

  default:
    Serial.println("0 = FAST flow, 1 = STANDARD Flow, 2 = ATTESTATION Flow");
    break;
  }
}

void setLogLevel(const char* buf) {
  esp_log_level_t level = esp_log_level_get("*");
  if (strncmp(buf + 1, "E", 1) == 0) {
    level = ESP_LOG_ERROR;
    Serial.println("ERROR");
  }
  else if (strncmp(buf + 1, "W", 1) == 0) {
    level = ESP_LOG_WARN;
    Serial.println("WARNING");
  }
  else if (strncmp(buf + 1, "I", 1) == 0) {
    level = ESP_LOG_INFO;
    Serial.println("INFO");
  }
  else if (strncmp(buf + 1, "D", 1) == 0) {
    level = ESP_LOG_DEBUG;
    Serial.println("DEBUG");
  }
  else if (strncmp(buf + 1, "V", 1) == 0) {
    level = ESP_LOG_VERBOSE;
    Serial.println("VERBOSE");
  }
  else if (strncmp(buf + 1, "N", 1) == 0) {
    level = ESP_LOG_NONE;
    Serial.println("NONE");
  }

  esp_log_level_set(TAG, level);
  esp_log_level_set("HK_HomeKit", level);
  esp_log_level_set("HKAuthCtx", level);
  esp_log_level_set("HKFastAuth", level);
  esp_log_level_set("HKStdAuth", level);
  esp_log_level_set("HKAttestAuth", level);
  esp_log_level_set("PN532", level);
  esp_log_level_set("PN532_SPI", level);
  esp_log_level_set("ISO18013_SC", level);
  esp_log_level_set("LockMechanism", level);
  esp_log_level_set("NFCAccess", level);
}

void print_issuers(const char* buf) {
  for (auto&& issuer : readerData.issuers) {
    LOG(I, "Issuer ID: %s, Public Key: %s", utils::bufToHexString(issuer.issuer_id.data(), issuer.issuer_id.size()).c_str(), utils::bufToHexString(issuer.issuer_pk.data(), issuer.issuer_pk.size()).c_str());
    for (auto&& endpoint : issuer.endpoints) {
      LOG(I, "Endpoint ID: %s, Public Key: %s", utils::bufToHexString(endpoint.endpoint_id.data(), endpoint.endpoint_id.size()).c_str(), utils::bufToHexString(endpoint.endpoint_pk.data(), endpoint.endpoint_pk.size()).c_str());
    }
  }
}

void notFound(AsyncWebServerRequest* request) {
  request->send(404, "text/plain", "Not found");
}

void listDir(fs::FS& fs, const char* dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("- failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    }
    else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

String miscHtmlProcess(const String& var) {
  if (var == "DEVICENAME") {
    return String(espConfig::miscConfig.deviceName.c_str());
  }
  else if (var == "OTAPASSWD") {
    return String(espConfig::miscConfig.otaPasswd.c_str());
  }
  else if (var == "HKSETUPCODE") {
    return String(espConfig::miscConfig.setupCode.c_str());
  }
  else if (var == "CONTROLPIN") {
    return String(espConfig::miscConfig.controlPin);
  }
  else if (var == "LEDPIN") {
    return String(espConfig::miscConfig.hsStatusPin);
  }
  else if (var == "NFCNEOPIXELPIN") {
    return String(espConfig::miscConfig.nfcNeopixelPin);
  }
  else if (var == "NFC1PIN") {
    return String(espConfig::miscConfig.nfcSuccessPin);
  }
  else if (var == "NFC2PIN") {
    return String(espConfig::miscConfig.nfcFailPin);
  }
  else if (var == "NFC1HL") {
    return String(espConfig::miscConfig.nfcSuccessHL);
  }
  else if (var == "NFC2HL") {
    return String(espConfig::miscConfig.nfcFailHL);
  }
  else if (var == "NFC1TIME") {
    return String(espConfig::miscConfig.nfcSuccessTime);
  }
  else if (var == "NFC2TIME") {
    return String(espConfig::miscConfig.nfcFailTime);
  }
  else if (var == "ALWAYSUNLOCK") {
    return String(espConfig::miscConfig.lockAlwaysUnlock);
  }
  else if (var == "ALWAYSLOCK") {
    return String(espConfig::miscConfig.lockAlwaysLock);
  }
  else if (var == "GPIOAEN") {
    return String(espConfig::miscConfig.gpioActionEnable);
  }
  else if (var == "GPIOAPIN") {
    return String(espConfig::miscConfig.gpioActionPin);
  }
  else if (var == "GPIOALOCK") {
    return String(espConfig::miscConfig.gpioActionLockState);
  }
  else if (var == "GPIOAUNLOCK") {
    return String(espConfig::miscConfig.gpioActionUnlockState);
  }
  else if (var == "HWFINISH") {
    return String(espConfig::miscConfig.hk_key_color);
  }
  return String();
}

String hkInfoHtmlProcess(const String& var) {
  String result = "";
  if (var == "READERGID") {
    return String(utils::bufToHexString(readerData.reader_gid.data(), readerData.reader_gid.size(), true).c_str());
  }
  else if (var == "READERID") {
    return String(utils::bufToHexString(readerData.reader_id.data(), readerData.reader_id.size(), true).c_str());
  }
  else if (var == "ISSUERSNO") {
    return String(readerData.issuers.size());
  }
  else if (var == "ISSUERSLIST") {
    for (auto&& issuer : readerData.issuers) {
      char issuerBuff[21 + 8];
      result += "<li>";
      snprintf(issuerBuff, sizeof(issuerBuff), "Issuer ID: %s", utils::bufToHexString(issuer.issuer_id.data(), issuer.issuer_id.size(), true).c_str());
      result += issuerBuff;
      result += "</li>\n";
      result += "\t\t<ul>";
      for (auto&& endpoint : issuer.endpoints) {
        char endBuff[23 + 6];
        result += "\n\t\t\t<li>";
        snprintf(endBuff, sizeof(endBuff), "Endpoint ID: %s", utils::bufToHexString(endpoint.endpoint_id.data(), endpoint.endpoint_id.size(), true).c_str());
        result += endBuff;
        result += "</li>\n";
      }
      result += "\t\t</ul>";
    }
    return result;
  }
  return result;
}

void setupWeb() {
  webServer.on("/info", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(LittleFS, "/info.html", "text/html", false, hkInfoHtmlProcess);
    });
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(LittleFS, "/index.html", "text/html", false, nullptr);
    });
  webServer.on("/misc", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(LittleFS, "/misc.html", "text/html", false, miscHtmlProcess);
    });
  webServer.onNotFound(notFound);
  webServer.begin();
}


bool timeractive = false;
int starttime = 0;
void checktimer() {
  if (timeractive == true) {
    if (millis() - starttime > 5000) {
      lockTargetState->setVal(lockStates::LOCKED);
      lockCurrentState->setVal(lockStates::LOCKED);
      timeractive = false;
      if (debug) Serial.print("Timer stopped");
      digitalWrite(RELAY_PIN, LOW);
    }
  }
}
void checkiftolongrunning() {
  if (millis() > 86400000) {
    ESP.restart();
  }
}

void nfc_thread_entry(void* arg) {
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    ESP_LOGE("NFC_SETUP", "Didn't find PN53x board");
  }
  else {
    unsigned int model = (versiondata >> 24) & 0xFF;
    ESP_LOGI("NFC_SETUP", "Found chip PN5%x", model);
    int maj = (versiondata >> 16) & 0xFF;
    int min = (versiondata >> 8) & 0xFF;
    ESP_LOGI("NFC_SETUP", "Firmware ver. %d.%d", maj, min);
    nfc.SAMConfig();
    nfc.setRFField(0x02, 0x01);
    nfc.setPassiveActivationRetries(0);
    ESP_LOGI("NFC_SETUP", "Waiting for an ISO14443A card");
  }
  memcpy(ecpData + 8, readerData.reader_gid.data(), readerData.reader_gid.size());
  with_crc16(ecpData, 16, ecpData + 16);
  while (1) {
    uint8_t res[4];
    uint16_t resLen = 4;
    nfc.writeRegister(0x633d, 0, true);
    nfc.inCommunicateThru(ecpData, sizeof(ecpData), res, &resLen, 100, true);
    uint8_t uid[16];
    uint8_t uidLen = 0;
    uint8_t atqa[2];
    uint8_t sak[1];
    bool passiveTarget = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, atqa, sak, 500, true, true);
    if (passiveTarget) {
      nfc.setPassiveActivationRetries(5);
      LOG(D, "ATQA: %02x", atqa[0]);
      LOG(D, "SAK: %02x", sak[0]);
      ESP_LOG_BUFFER_HEX_LEVEL(TAG, uid, (size_t)uidLen, ESP_LOG_VERBOSE);
      LOG(I, "*** PASSIVE TARGET DETECTED ***");
      auto startTime = std::chrono::high_resolution_clock::now();
      uint8_t data[13] = { 0x00, 0xA4, 0x04, 0x00, 0x07, 0xA0, 0x00, 0x00, 0x08, 0x58, 0x01, 0x01, 0x0 };
      uint8_t selectCmdRes[9];
      uint16_t selectCmdResLength = 9;
      LOG(D, "SELECT HomeKey Applet, APDU: ");
      ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, sizeof(data), ESP_LOG_VERBOSE);
      bool status = nfc.inDataExchange(data, sizeof(data), selectCmdRes, &selectCmdResLength);
      LOG(D, "SELECT HomeKey Applet, Response");
      ESP_LOG_BUFFER_HEX_LEVEL(TAG, selectCmdRes, selectCmdResLength, ESP_LOG_VERBOSE);
      if (status && selectCmdRes[selectCmdResLength - 2] == 0x90 && selectCmdRes[selectCmdResLength - 1] == 0x00) {
        LOG(D, "*** SELECT HOMEKEY APPLET SUCCESSFUL ***");
        LOG(D, "Reader Private Key: %s", utils::bufToHexString(readerData.reader_pk.data(), readerData.reader_pk.size()).c_str());
        HKAuthenticationContext authCtx(nfc, readerData, savedData);
        auto authResult = authCtx.authenticate(hkFlow);
        if (std::get<2>(authResult) != kFlowFailed) {
          bool status = true;
          xQueueSend(gpio_led_handle, &status, 0);
          if (espConfig::miscConfig.lockAlwaysUnlock) {
            lockCurrentState->setVal(lockStates::UNLOCKED);
            lockTargetState->setVal(lockStates::UNLOCKED);
            digitalWrite(RELAY_PIN, HIGH);
            starttime = millis();
            timeractive = true;
            if (debug) Serial.print("Timer started");
            Serial.print("Timer started");
            if (espConfig::miscConfig.gpioActionEnable && espConfig::miscConfig.gpioActionPin != 255) {
              digitalWrite(espConfig::miscConfig.gpioActionPin, espConfig::miscConfig.gpioActionUnlockState);
            }
          }
          else if (espConfig::miscConfig.lockAlwaysLock) {
            lockCurrentState->setVal(lockStates::LOCKED);
            lockTargetState->setVal(lockStates::LOCKED);
            if (espConfig::miscConfig.gpioActionEnable && espConfig::miscConfig.gpioActionPin != 255) {
              digitalWrite(espConfig::miscConfig.gpioActionPin, espConfig::miscConfig.gpioActionLockState);
            }
          }
          else {
            int targetState = lockTargetState->getNewVal();
            if (1 == 1) {
              if (targetState == lockStates::UNLOCKED) {
                digitalWrite(RELAY_PIN, HIGH);
                lockTargetState->setVal(lockStates::UNLOCKED);
                lockCurrentState->setVal(lockStates::UNLOCKED);
                starttime = millis();
                timeractive = true;
                if (debug) Serial.print("Timer started");
                Serial.print("Timer started");
              } else if (targetState == lockStates::LOCKED) {
                lockCurrentState->setVal(lockStates::LOCKED);
                lockTargetState->setVal(lockStates::LOCKED);
              }
            }
          }

          auto stopTime = std::chrono::high_resolution_clock::now();
          LOG(I, "Total Time (detection->auth->gpio): %lli ms", std::chrono::duration_cast<std::chrono::milliseconds>(stopTime - startTime).count());
        }
        else {
          bool status = false;
          xQueueSend(gpio_led_handle, &status, 0);
          LOG(W, "We got status FlowFailed!");
        }
        nfc.setRFField(0x02, 0x01);
      }
      else {
        LOG(I, "Invalid Response, probably not Homekey");
        bool status = true;
        xQueueSend(gpio_led_handle, &status, 0);
      }
      vTaskDelay(50 / portTICK_PERIOD_MS);
      nfc.inRelease();
      int counter = 50;
      bool deviceStillInField = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen);
      LOG(D, "Target still present: %d", deviceStillInField);
      while (deviceStillInField) {
        if (counter == 0) break;
        vTaskDelay(50 / portTICK_PERIOD_MS);
        nfc.inRelease();
        deviceStillInField = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen);
        --counter;
        LOG(D, "Target still present: %d Counter=%d", deviceStillInField, counter);
      }
      nfc.inRelease();
      nfc.setPassiveActivationRetries(0);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void gpio_task(void* arg) {
  bool status = false;
  while (1) {
    checktimer();
    checkiftolongrunning();
    if (gpio_led_handle != nullptr) {
      if (uxQueueMessagesWaiting(gpio_led_handle) > 0) {
        LOG(D, "Got something in queue");
        xQueueReceive(gpio_led_handle, &status, 0);
        if (status) {
          if (espConfig::miscConfig.nfcSuccessPin && espConfig::miscConfig.nfcSuccessPin != 255) {
            digitalWrite(espConfig::miscConfig.nfcSuccessPin, espConfig::miscConfig.nfcSuccessHL);
            delay(espConfig::miscConfig.nfcSuccessTime);
            digitalWrite(espConfig::miscConfig.nfcSuccessPin, !espConfig::miscConfig.nfcSuccessHL);
          }
          if (espConfig::miscConfig.nfcNeopixelPin && espConfig::miscConfig.nfcNeopixelPin != 255) {
            pixels.setPixelColor(0, pixels.Color(0, 255, 0));
            pixels.show();
            delay(espConfig::miscConfig.nfcSuccessTime);
            pixels.clear();
            pixels.show();
          }
        } else {
          if (espConfig::miscConfig.nfcFailPin && espConfig::miscConfig.nfcFailPin != 255) {
            digitalWrite(espConfig::miscConfig.nfcFailPin, espConfig::miscConfig.nfcFailHL);
            delay(espConfig::miscConfig.nfcFailTime);
            digitalWrite(espConfig::miscConfig.nfcFailPin, !espConfig::miscConfig.nfcFailHL);
          }
          if (espConfig::miscConfig.nfcNeopixelPin && espConfig::miscConfig.nfcNeopixelPin != 255) {
            pixels.setPixelColor(0, pixels.Color(255, 0, 0));
            pixels.show();
            delay(espConfig::miscConfig.nfcFailTime);
            pixels.clear();
            pixels.show();
          }
        }
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  const esp_app_desc_t* app_desc = esp_ota_get_app_description();
  std::string app_version = app_desc->version;
  gpio_led_handle = xQueueCreate(2, sizeof(bool));
  size_t len;
  const char* TAG = "SETUP";
  nvs_open("SAVED_DATA", NVS_READWRITE, &savedData);
  if (!nvs_get_blob(savedData, "READERDATA", NULL, &len)) {
    std::vector<uint8_t> savedBuf(len);
    nvs_get_blob(savedData, "READERDATA", savedBuf.data(), &len);
    LOG(I, "NVS DATA LENGTH: %d", len);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, savedBuf.data(), savedBuf.size(), ESP_LOG_DEBUG);
    try {
      readerData = msgpack::decode_msgpack<readerData_t>(savedBuf);
    }
    catch (const std::exception& e) {
      std::cerr << e.what() << '\n';
    }

  }
  if (!nvs_get_blob(savedData, "MISCDATA", NULL, &len)) {
    uint8_t msgpack[len];
    nvs_get_blob(savedData, "MISCDATA", msgpack, &len);
    std::string str(msgpack, msgpack + len);
    LOG(D, "MISCDATA - JSON(%d): %s", len, str.c_str());
    try {
      espConfig::miscConfig = decode_json<espConfig::misc_config_t>(str);
    }
    catch (const std::exception& e) {
      LOG(E, "%s", e.what());
    }
  }
  if (espConfig::miscConfig.nfcSuccessPin && espConfig::miscConfig.nfcSuccessPin != 255) {
    pinMode(espConfig::miscConfig.nfcSuccessPin, OUTPUT);
    digitalWrite(espConfig::miscConfig.nfcSuccessPin, !espConfig::miscConfig.nfcSuccessHL);
  }
  if (espConfig::miscConfig.nfcFailPin && espConfig::miscConfig.nfcFailPin != 255) {
    pinMode(espConfig::miscConfig.nfcFailPin, OUTPUT);
    digitalWrite(espConfig::miscConfig.nfcFailPin, !espConfig::miscConfig.nfcFailHL);
  }
  if (espConfig::miscConfig.gpioActionPin && espConfig::miscConfig.gpioActionPin != 255) {
    pinMode(espConfig::miscConfig.gpioActionPin, OUTPUT);
  }
  if (!LittleFS.begin(true)) {
    Serial.println("An Error has occurred while mounting LITTLEFS");
    return;
  }
  listDir(LittleFS, "/", 0);
  if (espConfig::miscConfig.controlPin != 255) {
    homeSpan.setControlPin(espConfig::miscConfig.controlPin);
  }
  if (espConfig::miscConfig.hsStatusPin != 255) {
   homeSpan.setStatusPin(espConfig::miscConfig.hsStatusPin);
  }
  homeSpan.setStatusAutoOff(15);
  homeSpan.reserveSocketConnections(2);
  homeSpan.setLogLevel(0);
  homeSpan.setSketchVersion(app_version.c_str());

  LOG(I, "READER GROUP ID (%d): %s", readerData.reader_gid.size(), utils::bufToHexString(readerData.reader_gid.data(), readerData.reader_gid.size()).c_str());
  LOG(I, "READER UNIQUE ID (%d): %s", readerData.reader_id.size(), utils::bufToHexString(readerData.reader_id.data(), readerData.reader_id.size()).c_str());

  LOG(I, "HOMEKEY ISSUERS: %d", readerData.issuers.size());
  for (auto&& issuer : readerData.issuers) {
    LOG(D, "Issuer ID: %s, Public Key: %s", utils::bufToHexString(issuer.issuer_id.data(), issuer.issuer_id.size()).c_str(), utils::bufToHexString(issuer.issuer_pk.data(), issuer.issuer_pk.size()).c_str());
  }
  homeSpan.enableAutoStartAP();
  homeSpan.enableOTA(espConfig::miscConfig.otaPasswd.c_str());
  homeSpan.setPortNum(1201);
  homeSpan.begin(Category::Locks, espConfig::miscConfig.deviceName.c_str(), "HK", "HomeKey-ESP32");
  homeSpan.autoPoll();

  new SpanUserCommand('D', "Delete Home Key Data", deleteReaderData);
  new SpanUserCommand('L', "Set Log Level", setLogLevel);
  new SpanUserCommand('F', "Set HomeKey Flow", setFlow);
  new SpanUserCommand('P', "Print Issuers", print_issuers);

  new SpanAccessory();
  new Service::AccessoryInformation();
  new Characteristic::Identify();
  new Characteristic::Manufacturer("rednblkx");
  new Characteristic::Model("HomeKey-ESP32");
  new Characteristic::Name(DEVICE_NAME);
  uint8_t mac[6];
  WiFi.macAddress(mac);
  pinMode(RELAY_PIN, OUTPUT);
  char macStr[18] = { 0 };
  sprintf(macStr, "%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3]);
  std::string serialNumber = "HK-";
  serialNumber.append(macStr);
  new Characteristic::SerialNumber(serialNumber.c_str());
  new Characteristic::FirmwareRevision(app_version.c_str());
  std::vector<uint8_t> decB64 = utils::decodeB64(hk_color_vals[HK_COLOR(espConfig::miscConfig.hk_key_color)]);
  TLV8 hwfinish(NULL, 0);
  hwfinish.unpack(decB64.data(), decB64.size());
  new Characteristic::HardwareFinish(hwfinish);

  new LockManagement();
  new LockMechanism();
  new NFCAccess();
  new Service::HAPProtocolInformation();
  new Characteristic::Version();
  homeSpan.setControllerCallback(pairCallback);
  homeSpan.setWifiCallback([]() { setupWeb(); });

  if (espConfig::miscConfig.nfcNeopixelPin && espConfig::miscConfig.nfcNeopixelPin != 255) {
    pixels.setPin(espConfig::miscConfig.nfcNeopixelPin);
    pixels.begin();
  }

  xTaskCreate(gpio_task, "gpio_task", 4096, NULL, 1, NULL);
  xTaskCreate(nfc_thread_entry, "nfc_task", 8192, NULL, 2, NULL);
}

//////////////////////////////////////

void loop() {
}
