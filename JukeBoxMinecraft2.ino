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

#define RFID_SCK  19  
#define RFID_MISO 15  
#define RFID_MOSI 22 
#define RFID_SS   23
#define RFID_RST  4

#define SD_SCK  17
#define SD_MISO 5
#define SD_MOSI 18 
#define SD_CS   2

#define I2S_BCLK 25
#define I2S_LRC  26
#define I2S_DIN  33

// ---------------- SPI BUS ----------------
SPIClass SPI_SD(VSPI);
SPIClass SPI_RFID(HSPI);
MFRC522 rfid(RFID_SS, RFID_RST, &SPI_RFID);

// ---------------- GLOBAL ----------------
unsigned long lastPress = 0;
volatile bool btnPressed = false;
unsigned long pressStart = 0;
bool btnPrev = HIGH;

// Mode: 0 = RFID, 1 = Bluetooth
volatile int mode = 0; 

unsigned long nextPressStart = 0;
unsigned long prevPressStart = 0;
bool nextPrev = HIGH;
bool prevPrev = HIGH;

int volumeLevel = 50;   // BT Volume
float sdVolume = 1.0f;  // SD Volume

File file;
bool isPlaying = false;
byte lastUID[10] = {0,0,0,0,0,0,0,0,0,0};
bool isManuallyPaused = false;
uint8_t buffer[512];
size_t bytes_written;

unsigned long lastSeen = 0;
const unsigned long CARD_TIMEOUT = 1500;

// ---------------- A2DP ----------------
I2SStream i2s_bt;
BluetoothA2DPSink a2dp_sink(i2s_bt);

// ---------------- UID Mapping ----------------
byte style[4]      = {0x55, 0x9C, 0xD1, 0x49};
byte shakeitoff[4] = {0xAE, 0x4F, 0x37, 0x06};
byte blankspace[4] = {0x9C, 0x4C, 0x3A, 0x06};

// ---------------- PROTOTYPES ----------------
void fadeOut();
void injectSilence();
void i2s_init_sd();

// ---------------- FUNC ----------------
// เปลี่ยนชื่อฟังก์ชันเพื่อให้สื่อสารว่าเปรียบเทียบ 7 ไบต์
bool isSameUID_7Byte(byte *a, byte *b){ 
   for(int i=0;i<7;i++) if(a[i]!=b[i]) return false;
   return true;
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

// ฟังก์ชันยัดเสียงเงียบและล้าง DMA
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
void i2s_init_sd(){
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
  if(isPlaying) fadeOut();
  isPlaying = false;
  if(file) file.close();
  injectSilence();
  i2s_driver_uninstall(I2S_NUM_0); //ล้างขาเพื่อไม่ให้ i2s ชนกัน
  delay(200);

  // 3. เริ่ม Bluetooth
  auto cfg = i2s_bt.defaultConfig();
  cfg.pin_bck = I2S_BCLK;
  cfg.pin_ws  = I2S_LRC;
  cfg.pin_data = I2S_DIN;
  i2s_bt.begin(cfg);
  
  a2dp_sink.set_volume(volumeLevel);
  a2dp_sink.start("MIFI");
  
  mode = 1;
  Serial.println("✅ Bluetooth Mode Ready");
}

// เข้าโหมด RFID / SD
void enterModeRFID() {
  Serial.println("🔄 Switching to RFID/SD...");
  
  // 1. หยุด Bluetooth และคืนทรัพยากร
  a2dp_sink.stop();
  a2dp_sink.end(true); // true = release i2s
  delay(200);

  // 2. เริ่ม Driver SD ใหม่
  i2s_init_sd();
  mode = 0;
  isPlaying = false;
  isManuallyPaused = false;  
  // ล้าง UID ล่าสุด เพื่อให้บัตรเดิมที่แตะอีกครั้งถูกมองว่าเป็น "บัตรใหม่" 
  memset(lastUID, 0, 4);    
  Serial.println("✅ RFID Mode Ready");
}

void openFileForUID(byte *uid){
  if(isPlaying) {
      fadeOut();
      isPlaying = false;
      injectSilence();
  }

  if(file) file.close();

  if(isSameUID_7Byte(uid, style)){
    file = SD.open("/music/Taylor-Swift-style.wav");
    Serial.println("Taylor-Swift-style");
  }
  else if(isSameUID_7Byte(uid, shakeitoff)){
    file = SD.open("/music/Taylor-Swift-shakeitoff.wav");
    Serial.println("Taylor-Swift-shakeitoff");
  }
  else if(isSameUID_7Byte(uid, blankspace)){
    file = SD.open("/music/Taylor-Swift-blankSpace.wav");
    Serial.println("Taylor-Swift-blankSpace");
  }
  if(!file){
    Serial.println("❌ File open failed");
    return;
  }

  file.seek(44);
  // *** คัดลอก UID 7 ไบต์จริง ๆ ***
  memcpy(lastUID, uid, rfid.uid.size); 
  // *** ล้างส่วนที่เหลือให้เป็น 0 (เผื่อบัตร 4 ไบต์) ***
  if (rfid.uid.size < 10) {
      memset(&lastUID[rfid.uid.size], 0, 10 - rfid.uid.size);
  }
  isPlaying = true;
  Serial.println("🎵 New song selected");
}

// ---------------- TASK RFID ----------------
void TaskRFID(void *pv){
  while(1){
    if(mode == 1){ 
      vTaskDelay(200);
      continue;
    }

    if(rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()){
      lastSeen = millis();
      
      // *** 1. ตรวจสอบชนิดบัตร (PICC Type) ***
      MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
      if (piccType != MFRC522::PICC_TYPE_MIFARE_UL && // NTAG213/215
          piccType != MFRC522::PICC_TYPE_MIFARE_1K && // MIFARE Classic
          piccType != MFRC522::PICC_TYPE_MIFARE_4K) 
      {
          // ไม่ใช่ชนิดที่รองรับ ให้ข้ามไป
          rfid.PICC_HaltA();
          vTaskDelay(50);
          continue;
      }

      // *** 2. เปรียบเทียบ UID (ตามขนาดจริงที่อ่านได้) ***
      bool isSame = true;
      // วนลูปเปรียบเทียบตามขนาดจริงที่อ่านได้จากบัตร (4 หรือ 7 ไบต์)
      for (int i = 0; i < rfid.uid.size; i++) {
          if (rfid.uid.uidByte[i] != lastUID[i]) {
              isSame = false;
              break;
          }
      }

      if(isSame){ // บัตรเดิม
        if (!isPlaying && !isManuallyPaused) {
             isPlaying = true;
        }
        rfid.PICC_HaltA(); // หยุดการอ่าน
        vTaskDelay(50);
        continue;
      }
      
      // บัตรใหม่
      openFileForUID(rfid.uid.uidByte);
      rfid.PICC_HaltA(); // หยุดการอ่าน
    }
    
    // ... ส่วน Timeout เหมือนเดิม
    vTaskDelay(50);
  }
}

// ---------------- TASK AUDIO ----------------
void TaskAudio(void *pv){
  while(1){
    if(mode == 1){
      vTaskDelay(100);
      continue;
    }
    // SD Player Mode
    if(!isPlaying){
      vTaskDelay(10);
      continue;
    }

    if(isPlaying && file){
      if(file.available()){
        size_t bytes_read = file.read(buffer, sizeof(buffer));

        // --- Digital Volume Scaling ---
        int16_t *samples = (int16_t*)buffer;  
        int count = bytes_read / 2;
        for(int i=0; i<count; i++){
            samples[i] = samples[i] * sdVolume;
        }

        size_t bytes_out;
        i2s_write(I2S_NUM_0, buffer, bytes_read, &bytes_out, portMAX_DELAY);

      } else {
        // เพลงจบแล้ว วนลูป หรือ หยุด
        file.seek(44); // Loop
      }
    }
    vTaskDelay(1);
  }
}

// =========================
// Play / Stop / Volume
// =========================
void playAudio() {
  if (mode == 1) {
    Serial.println("▶️ BT Play Request");
    a2dp_sink.play(); // สั่ง BT เล่น
    isManuallyPaused = false; 
  } else {
    if (file) {
        Serial.println("▶️ SD Play Request");
        // file.seek(44);
        isPlaying = true;
        isManuallyPaused = false; 
    }
  }
}

void stopAudio() {
  if (mode == 1) {
    Serial.println("⏸️ BT Pause Request");
    a2dp_sink.pause(); // สั่ง BT หยุดชั่วคราว
    isManuallyPaused = true; 
  } else {
    Serial.println("⏸️ SD Stop Request");
    fadeOut();
    isPlaying = false;
    isManuallyPaused = true; 
    injectSilence();
  }
}

void volumeUp(){
  volumeLevel += 5;
  if(volumeLevel > 100) volumeLevel = 100;
  a2dp_sink.set_volume(volumeLevel);
  Serial.printf("🔊 Vol: %d\n", volumeLevel);
}

void volumeDown(){
  volumeLevel -= 5;
  if(volumeLevel < 0) volumeLevel = 0;
  a2dp_sink.set_volume(volumeLevel);
  Serial.printf("🔉 Vol: %d\n", volumeLevel);
}

void sdVolumeUp(){
  sdVolume += 0.1f;
  if(sdVolume > 1.0f) sdVolume = 1.0f;
  Serial.printf("🔊 SD Vol: %.1f\n", sdVolume);
}

void sdVolumeDown(){
  sdVolume -= 0.1f;
  if(sdVolume < 0.0f) sdVolume = 0.0f;
  Serial.printf("🔉 SD Vol: %.1f\n", sdVolume);
}

// ---------------- SETUP ----------------
void setup(){
  // ป้องกัน Brownout
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // กันไม่ให้ reboot เองตอนไฟตกหรือกระแสรไม่พอ
  Serial.begin(115200);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_PREV, INPUT_PULLUP);
  // SD
  SPI_SD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if(!SD.begin(SD_CS, SPI_SD)){
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
  xTaskCreatePinnedToCore(TaskRFID,  "RFID",  4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskAudio, "AUDIO", 4096, NULL, 1, NULL, 0);

  Serial.println("System Ready!");
}

// ---------------- LOOP (Button Logic) ----------------
void loop() {
  // --- MODE / PLAY / PAUSE BUTTON ---
  bool btnNow = digitalRead(BTN_PIN);
  if (btnPrev == HIGH && btnNow == LOW) {
    pressStart = millis();
  }
  if (btnPrev == LOW && btnNow == HIGH) {
    unsigned long pressTime = millis() - pressStart;

    if (pressTime < 800) {
      // --- Short Press: Toggle Play/Pause ---
      if (mode == 1) {
          // ในโหมด BT เราเช็คสถานะยาก ให้ toggle เอาเลย หรือสั่ง play/pause
          // แต่ A2DP Sink เช็คสถานะลำบาก สมมติว่าสั่งสลับกัน
          // *ถ้าจะให้แม่นยำ ต้องดู state จาก library แต่กดซ้ำๆ ก็ work ครับ
          if(a2dp_sink.get_audio_state() == ESP_A2D_AUDIO_STATE_STARTED){
            stopAudio();
          }else {
            playAudio();
          }
      } else {
          // โหมด SD
          if (isPlaying) stopAudio();
          else playAudio();
      }

    } else {
      // --- Long Press: Switch Mode ---
      if (mode == 0) {
        enterModeBluetooth();
      } else {
        enterModeRFID();
      }
    }
  }
  btnPrev = btnNow;

  // --- NEXT BUTTON ---
  bool nextNow = digitalRead(BTN_NEXT);
  if(nextPrev == HIGH && nextNow == LOW) nextPressStart = millis();
  if(nextPrev == LOW && nextNow == HIGH){
    if((millis() - nextPressStart) < 500){
       if(mode==1) volumeUp(); else sdVolumeUp();
    } else {
       if(mode==1) a2dp_sink.next();
    }
  }
  nextPrev = nextNow;

  // --- PREV BUTTON ---
  bool prevNow = digitalRead(BTN_PREV);
  if(prevPrev == HIGH && prevNow == LOW) prevPressStart = millis();
  if(prevPrev == LOW && prevNow == HIGH){
    if((millis() - prevPressStart) < 500){
       if(mode==1) volumeDown(); else sdVolumeDown();
    } else {
       if(mode==1) a2dp_sink.previous();
    }
  }
  prevPrev = prevNow;  
}