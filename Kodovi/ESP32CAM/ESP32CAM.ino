#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// Kreiranje serijske komunikacije ka glavnom ESP32S
HardwareSerial camSerial(1);

// ====== CAMERA PINOUT AI THINKER ======
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  
  // Koristimo 10MHz za stabilnost komunikacije bez PSRAM-a
  config.xclk_freq_hz = 10000000; 
  config.pixel_format = PIXFORMAT_RGB565; 
  config.frame_size = FRAMESIZE_96X96;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) return;
  
  sensor_t * s = esp_camera_sensor_get();
  s->set_saturation(s, 2); // Jace boje
  s->set_contrast(s, 1);   // Bolji ivice
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  
  // Inicijalizacija serijske komunikacije (TX pin je IO12)
  camSerial.begin(115200, SERIAL_8N1, -1, 12); 
  
  // Standardni Serial za tvoj monitoring na laptopu
  Serial.begin(115200);
  
  startCamera();
  Serial.println("Kamera spremna. Saljem podatke na IO12...");
}

void loop() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) return;

  long rSum = 0, gSum = 0, bSum = 0;
  
  // PROZOR DETEKCIJE (30x30 u centru za vecu udaljenost)
  int prozor = 30; 
  int startX = (fb->width - prozor) / 2;
  int startY = (fb->height - prozor) / 2;
  int count = 0;

  for (int y = startY; y < startY + prozor; y++) {
    for (int x = startX; x < startX + prozor; x++) {
      int offset = (y * fb->width + x) * 2;
      // RGB565 raspakivanje
      uint16_t pixel = (fb->buf[offset] << 8) | fb->buf[offset+1];
      rSum += ((pixel >> 11) & 0x1F) << 3;
      gSum += ((pixel >> 5) & 0x3F) << 2;
      bSum += (pixel & 0x1F) << 3;
      count++;
    }
  }

  float r = rSum / (float)count;
  float g = gSum / (float)count;
  float b = bSum / (float)count;

  // Logika detekcije (isto kao u prethodnom stabilnom resenju)
  float threshold = 1.15; 

  if (g > r * threshold && g > b) {
    camSerial.println("G"); // Salje ESP32S-u da je Zeleno
    Serial.println("Detektovano: ZELENO");
  } 
  else if (r > g * threshold && r > b) {
    camSerial.println("R"); // Salje ESP32S-u da je Crveno
    Serial.println("Detektovano: CRVENO");
  } 
  else {
    camSerial.println("N"); // Nista (slobodan put)
    Serial.println("Detektovano: NISTA");
  }

  esp_camera_fb_return(fb);
  
  // Mala pauza da ne preopteretimo serijski bafer glavnog ESP32S
  delay(100); 
}