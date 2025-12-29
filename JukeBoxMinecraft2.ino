#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <SD.h>
#include <driver/i2s.h>
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ---------------- PIN ----------------
#define BTN_PIN 13
#define BTN_NEXT 27
#define BTN_PREV 14

#define RFID_SCK 19
#define RFID_MISO 15
#define RFID_MOSI 22
#define RFID_SS 23
#define RFID_RST 4

#define SD_SCK 17
#define SD_MISO 5
#define SD_MOSI 18
#define SD_CS 2

#define I2S_BCLK 25
#define I2S_LRC 26
#define I2S_DIN 33

// ---------------- SPI BUS ----------------
SPIClass SPI_SD(VSPI);
SPIClass SPI_RFID(HSPI);
MFRC522 rfid(RFID_SS, RFID_RST, &SPI_RFID);

// ---------------- GLOBAL ----------------
unsigned long lastPress = 0;
volatile bool btnPressed = false;
unsigned long pressStart = 0;
bool btnPrev = HIGH;

volatile int mode = 0;// Mode: 0 = RFID, 1 = Bluetooth

unsigned long nextPressStart = 0;
unsigned long prevPressStart = 0;
bool nextPrev = HIGH;
bool prevPrev = HIGH;

int volumeLevel = 50;   // BT Volume
float sdVolume = 1.0f;  // SD Volume

File file;
bool isPlaying = false;
byte lastUID[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
bool isManuallyPaused = false;
uint8_t buffer[512];
size_t bytes_written;

unsigned long lastSeen = 0;
const unsigned long CARD_TIMEOUT = 1500;

bool btPlaying = false;

unsigned long lastNextAction = 0;
unsigned long lastPrevAction = 0;
#define BTN_LOCK_MS 300

#define LONG_PRESS_MS 1500
#define DEBOUNCE_MS 40
bool firstBtAction = true;  // ป้องกัน false trigger ตอนกดครั้งแรกในโหมด Bluetooth


// ---------------- A2DP ----------------
I2SStream i2s_bt;
BluetoothA2DPSink a2dp_sink(i2s_bt);

// ---------------- UID Mapping ----------------
byte blankspace[7] = { 0x04, 0xB1, 0x03, 0x21, 0x3D, 0x41, 0x89 };
byte shakeitoff[7] = { 0x04, 0x71, 0xD4, 0x23, 0x3D, 0x41, 0x89 };
byte style[4] = { 0x9C, 0x4C, 0x3A, 0x06 };

// ---------------- PROTOTYPES ----------------
void fadeOut();
void injectSilence();
void i2s_init_sd();

// ---------------- FUNC ----------------
bool isSameUID_7Byte(byte *a, byte *b) {
  for (int i = 0; i < 7; i++)
    if (a[i] != b[i]) return false;
  return true;
}

bool isCardStillPresent() {
  byte atqa[2];
  byte size = sizeof(atqa);

  MFRC522::StatusCode status =
    rfid.PICC_WakeupA(atqa, &size);

  return (status == MFRC522::STATUS_OK);
}

// ฟังก์ชันค่อยๆ หรี่เสียง (Software Fade Out)
void fadeOut() {
  if (!isPlaying) return;
  float originalVol = sdVolume;
  for (float v = originalVol; v >= 0; v -= 0.1f) {
    sdVolume = v;
    vTaskDelay(20);
  }
  sdVolume = 0.0f;
  vTaskDelay(50);
  sdVolume = originalVol;
}

// ฟังก์ชันกันเสียง noice
void injectSilence() {
  uint8_t silenceBuffer[512];
  memset(silenceBuffer, 0, sizeof(silenceBuffer));
  size_t bytes_out;
  for (int i = 0; i < 5; i++) {
    i2s_write(I2S_NUM_0, silenceBuffer, sizeof(silenceBuffer), &bytes_out, 100);
    delay(5);
  }
  i2s_zero_dma_buffer(I2S_NUM_0);
}

// ตั้งค่า I2S สำหรับ SD Card (Manual Driver)
void i2s_init_sd() {
  // ต้อง Uninstall ก่อนเสมอเผื่อมี config เก่าค้าง
  i2s_driver_uninstall(I2S_NUM_0);
  delay(100);

  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,  // เพิ่ม Buffer หน่อยเพื่อความลื่น
    .dma_buf_len = 256
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DIN,
    .data_in_num = -1
  };

  i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

// เข้าโหมด Bluetooth
void enterModeBluetooth() {
  Serial.println("🔄 Switching to Bluetooth...");

  // 1. หยุด SD
  if (isPlaying) fadeOut();
  isPlaying = false;
  if (file) file.close();
  injectSilence();
  i2s_driver_uninstall(I2S_NUM_0);
  delay(100);
  i2s_bt.end(); 
  auto cfg = i2s_bt.defaultConfig();
  cfg.pin_bck = I2S_BCLK;
  cfg.pin_ws = I2S_LRC;
  cfg.pin_data = I2S_DIN;
  cfg.buffer_count = 8;
  cfg.buffer_size = 256; // เพิ่ม buffer ให้ BT เสถียรขึ้น
  i2s_bt.begin(cfg);
  a2dp_sink.end(false); 
  a2dp_sink.set_volume(volumeLevel);
  a2dp_sink.start("MIFI");

  mode = 1;
  firstBtAction = true;
  btPlaying = true;
  Serial.println("✅ Bluetooth Mode Ready");
}

// เข้าโหมด RFID / SD
void enterModeRFID() {
  Serial.println("🔄 Switching to RFID/SD...");
  if (a2dp_sink.get_audio_state() == ESP_A2D_AUDIO_STATE_STARTED) {
      a2dp_sink.stop();
  }
  a2dp_sink.disconnect();
  delay(100);
  a2dp_sink.end(false); 
  i2s_bt.end();
  i2s_driver_uninstall(I2S_NUM_0);
  delay(500);
  i2s_init_sd();
  
  mode = 0;
  isPlaying = false;
  isManuallyPaused = false;
  memset(lastUID, 0, 10);
  Serial.println("✅ RFID Mode Ready");
}

void openFileForUID(byte *uid) {
  if (isPlaying) {
    fadeOut();
    isPlaying = false;
    injectSilence();
  }

  if (file) file.close();

  if (isSameUID_7Byte(uid, style)) {
    file = SD.open("/music/Taylor-Swift-style.wav");
    Serial.println("Taylor-Swift-style");
  } else if (isSameUID_7Byte(uid, shakeitoff)) {
    file = SD.open("/music/Taylor-Swift-shakeitoff.wav");
    Serial.println("Taylor-Swift-shakeitoff");
  } else if (isSameUID_7Byte(uid, blankspace)) {
    file = SD.open("/music/Taylor-Swift-blankSpace.wav");
    Serial.println("Taylor-Swift-blankSpace");
  }
  if (!file) {
    Serial.println("❌ File open failed");
    return;
  }

  file.seek(44);
  memcpy(lastUID, uid, rfid.uid.size);
  if (rfid.uid.size < 10) {
    memset(&lastUID[rfid.uid.size], 0, 10 - rfid.uid.size);
  }
  isPlaying = true;
  Serial.println("🎵 New song selected");
}

// ---------------- TASK RFID ----------------
void TaskRFID(void *pv) {
  while (1) {
    if (mode == 1) {
      vTaskDelay(200);
      continue;
    }

    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      lastSeen = millis();

      // *** 1. ตรวจสอบชนิดบัตร (PICC Type) ***
      MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
      if (piccType != MFRC522::PICC_TYPE_MIFARE_UL &&  // NTAG213/215
          piccType != MFRC522::PICC_TYPE_MIFARE_1K &&  // MIFARE Classic
          piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
        // ไม่ใช่ชนิดที่รองรับ ให้ข้ามไป
        rfid.PICC_HaltA();
        vTaskDelay(50);
        continue;
      }

      // *** 2. เปรียบเทียบ UID (ตามขนาดจริงที่อ่านได้) ***
      bool isSame = true;
      for (int i = 0; i < rfid.uid.size; i++) {
        if (rfid.uid.uidByte[i] != lastUID[i]) {
          isSame = false;
          break;
        }
      }

      if (isSame) {  // บัตรเดิม
        if (!isPlaying && !isManuallyPaused) {
          isPlaying = true;
        }
        rfid.PICC_HaltA();  // หยุดการอ่าน
        vTaskDelay(50);
        continue;
      }

      // บัตรใหม่
      openFileForUID(rfid.uid.uidByte);
      rfid.PICC_HaltA();  // หยุดการอ่าน
    }
    //
    if (isPlaying && !isCardStillPresent()) {
      Serial.println("🟥 Card removed");
      fadeOut();
      isPlaying = false;
      injectSilence();
    }
    vTaskDelay(50);
  }
}

// ---------------- TASK AUDIO ----------------
void TaskAudio(void *pv) {
  while (1) {
    if (mode == 1) {
      vTaskDelay(100);
      continue;
    }
    // SD Player Mode
    if (!isPlaying) {
      vTaskDelay(10);
      continue;
    }

    if (isPlaying && file) {
      if (file.available()) {
        size_t bytes_read = file.read(buffer, sizeof(buffer));
        int16_t *samples = (int16_t *)buffer;
        int count = bytes_read / 2;
        for (int i = 0; i < count; i++) {
          samples[i] = samples[i] * sdVolume;
        }

        size_t bytes_out;
        i2s_write(I2S_NUM_0, buffer, bytes_read, &bytes_out, portMAX_DELAY);

      } else {
        // เพลงจบแล้ว วนลูป หรือ หยุด
        file.seek(44);  // Loop
      }
    }
    vTaskDelay(1);
  }
}

void playAudio() {
  if (mode == 1) {
    Serial.println("▶️ BT Play Request");
    a2dp_sink.play();
    btPlaying = true;
    isManuallyPaused = false;
  } else {
    if (file) {
      Serial.println("▶️ SD Play Request");
      isPlaying = true;
      isManuallyPaused = false;
    }
  }
}

void stopAudio() {
  if (mode == 1) {
    Serial.println("⏸️ BT Pause Request");
    a2dp_sink.pause();
    btPlaying = false;
    isManuallyPaused = true;
  } else {
    Serial.println("⏸️ SD Stop Request");
    fadeOut();
    isPlaying = false;
    isManuallyPaused = true;
    injectSilence();
  }
}


void volumeUp() {
  volumeLevel += 5;
  if (volumeLevel > 100) volumeLevel = 100;
  a2dp_sink.set_volume(volumeLevel);
  Serial.printf("🔊 Vol: %d\n", volumeLevel);
}

void volumeDown() {
  volumeLevel -= 5;
  if (volumeLevel < 0) volumeLevel = 0;
  a2dp_sink.set_volume(volumeLevel);
  Serial.printf("🔉 Vol: %d\n", volumeLevel);
}

void sdVolumeUp() {
  sdVolume += 0.1f;
  if (sdVolume > 1.0f) sdVolume = 1.0f;
  Serial.printf("🔊 SD Vol: %.1f\n", sdVolume);
}

void sdVolumeDown() {
  sdVolume -= 0.1f;
  if (sdVolume < 0.0f) sdVolume = 0.0f;
  Serial.printf("🔉 SD Vol: %.1f\n", sdVolume);
}

// ---------------- SETUP ----------------
void setup() {
  // ป้องกัน Brownout
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // กันไม่ให้ reboot เองตอนไฟตกหรือกระแสรไม่พอ
  Serial.begin(115200);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_PREV, INPUT_PULLUP);
  // SD
  SPI_SD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI_SD)) {
    Serial.println("❌ SD Mount Failed");
  } else {
    Serial.println("✅ SD Mounted");
  }

  // RFID
  SPI_RFID.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
  rfid.PCD_Init();
  // เริ่มต้นด้วยโหมด SD/RFID
  Serial.println("Starting in RFID Mode...");
  mode = 0;
  i2s_init_sd();
  // Tasks
  xTaskCreatePinnedToCore(TaskRFID, "RFID", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskAudio, "AUDIO", 4096, NULL, 1, NULL, 0);

  Serial.println("System Ready!");
}

// ---------------- LOOP (Button Logic) ----------------
void loop() {
  // ===== PLAY / PAUSE / MODE BUTTON =====
  static bool playPressed = false;
  static bool playHandled = false;
  static unsigned long playPressTime = 0;
  static unsigned long playReleaseTime = 0;

  bool playNow = (digitalRead(BTN_PIN) == LOW);  // Active LOW

  // เริ่มกด
  if (!playPressed && playNow && millis() - playReleaseTime > 150) {
    playPressed = true;
    playHandled = false;
    playPressTime = millis();
  }

  // ระหว่างกดค้าง
  if (playPressed && playNow && !playHandled) {
    unsigned long held = millis() - playPressTime;
    if (held >= 800) {
      // ---- LONG PRESS → SWITCH MODE ----
      if (mode == 0) enterModeBluetooth();
      else enterModeRFID();
      playHandled = true;  // กันไม่ให้ไป trigger short press
    }
  }

  // ปล่อยนิ้ว
  if (playPressed && !playNow) {
    playPressed = false;
    playReleaseTime = millis();

    if (!playHandled) {
      unsigned long pressDuration = millis() - playPressTime;
      if (pressDuration >= 50 && pressDuration < 800) {
        // ---- SHORT PRESS → PLAY / PAUSE ----
        if (mode == 1) {
          if (btPlaying) stopAudio();
          else playAudio();
        } else {
          if (isPlaying) stopAudio();
          else playAudio();
        }
      }
    }
  }

    // --- NEXT BUTTON ---
  bool nextNow = digitalRead(BTN_NEXT);
  if (nextPrev == LOW && nextNow == HIGH) {
    nextPressStart = millis();
  }
  if (nextPrev == HIGH && nextNow == LOW) {
    unsigned long pressTime = millis() - nextPressStart;
    if (mode == 1) { // Bluetooth mode
      if (pressTime >= 800) {
        Serial.println("⏭️ BT Next Track");
        a2dp_sink.next();
      } else if (pressTime >= 50) {
        Serial.println("🔊 BT Volume Up (short press)");
        volumeUp();
      }
    } else if (mode == 0) { // RFID/SD mode
      if (pressTime >= 50) {
        Serial.println("🔊 SD Volume Up (short press)");
        sdVolumeUp();
      }
    }
  }
  nextPrev = nextNow;

  // --- PREV BUTTON ---
  bool prevNow = digitalRead(BTN_PREV);
  if (prevPrev == LOW && prevNow == HIGH) {
    prevPressStart = millis();
  }
  if (prevPrev == HIGH && prevNow == LOW) {
    unsigned long pressTime = millis() - prevPressStart;
    if (mode == 1) { // Bluetooth mode
      if (pressTime >= 800) {
        Serial.println("⏮️ BT Previous Track");
        a2dp_sink.previous();
      } else if (pressTime >= 50) {
        Serial.println("🔉 BT Volume Down (short press)");
        volumeDown();
      }
    } else if (mode == 0) { // RFID/SD mode
      if (pressTime >= 50) {
        Serial.println("🔉 SD Volume Down (short press)");
        sdVolumeDown();
      }
    }
  }
  prevPrev = prevNow;

}