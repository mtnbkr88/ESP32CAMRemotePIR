/**********************************************************************************
 * 11/09/2020 Edward Williams
 * Upon start this sketch will check if start was from a PIR signal or the deep 
 * timer/power on/reset).
 * 
 * If wakeup was from the PIR this sketch will send a remote trigger command to the 
 * remoteTriggerIP then, based on settings, take and email pictures or take and 
 * email a short video. Then it will go into a deep sleep. 
 * 
 * If wakeup was from the deep sleep timer/power on/reset then start a web server 
 * for 10 minutes to allow changing configuration settings or update firmware.
 * Then it will go into a deep sleep.
 * 
 * A battery divider circuit with a 147K and 100K resister will be attached to GPIO12
 * for battery voltage reads. A PIR detector will be attached to GPIO13 for PIR wakeup
 * from deep sleep.
 **********************************************************************************/


/**********************************************************************************
 * 03/25/2020 Edward Williams
 * This is a shell which includes an Over-The-Air firmware update web server which
 * includes the option to erase EEPROM, fixed IP address, a major fail flashing led 
 * notice with sleep reboot, time set and mount of SD card. I use this as a starting 
 * point for all my sketches. Visit the web site <IP>/updatefirmware and enter the 
 * password to upload new firmware to the ESP32. Compile using the "Default" Partition 
 * Scheme.
 **********************************************************************************/
 
// edit the below for local settings 

// wifi info
const char* ssid = "YourSSID";
const char* password = "YourSSIDPwd";
// fixed IP info
const uint8_t IP_Address[4] = {192, 168, 2, 30};
const uint8_t IP_Gateway[4] = {192, 168, 2, 1};
const uint8_t IP_Subnet[4] = {255, 255, 255, 0};
const uint8_t IP_PrimaryDNS[4] = {8, 8, 8, 8};
const uint8_t IP_SecondaryDNS[4] = {8, 8, 4, 4};

const char* TZ_INFO = "PST8PDT,M3.2.0/2:00:00,M11.1.0/2:00:00";

const int SERVER_PORT = 80;  // port the main web server will listen on

// edit email server info for the send part, recipient address is set through Settings in the app
const char* emailhost = "smtp.gmail.com";
const int emailport = 465;
const char* emailsendaddr = "YourEmail\@gmail.com";
const char* emailsendpwd = "YourEmailPwd";
char email[40] = "DefaultMotionDetectEmail\@hotmail.com";  // this can be changed through Settings in the app

const char* appName = "ESP32CamRemotePIR";
const char* appVersion = "1.0.4";
const char* firmwareUpdatePassword = "87654321";

// should not need to edit the below

#include "esp_http_server.h"
httpd_handle_t webserver_httpd = NULL;   

#include <WiFi.h>

#include "soc/soc.h"  //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems

#include "time.h"
#include "lwip/err.h"
#include "lwip/apps/sntp.h"
struct tm timeinfo;
time_t now;

#include <EEPROM.h>  // for erasing EEPROM ATO, assumes EEPROM is 512 bytes in size
#include <Update.h>  // for flashing new firmware

#define uS_TO_S_FACTOR 1000000LL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP1S  1        // Time ESP32 will go to sleep (in seconds)

// EEPROM setup for saving values over reboots
#include <EEPROM.h>  // also used in OTA firmware update screen
#define EEPROM_READY_ADDR 0x00
const byte eeprom_ready[] = {0x52, 0x65, 0x61, 0x64, 0x79};
#define EEPROM_REMOTE_ADDR (EEPROM_READY_ADDR+sizeof(eeprom_ready))
int remoteTrigger = 0;  // <10 is off, >10 is on
#define EEPROM_REMOTEIP_ADDR (EEPROM_REMOTE_ADDR+sizeof(remoteTrigger))
char remoteTriggerIP[16] = "0.0.0.0";  // example
#define EEPROM_EMAIL_ADDR (EEPROM_REMOTEIP_ADDR+sizeof(remoteTriggerIP))
// default email defined above
#define EEPROM_MOTIONEMAILACTION_ADDR (EEPROM_EMAIL_ADDR+sizeof(email))
int motionEmailAction = 1;  // < 10 is off, >10 is on
                            // 1(11) to 6(16) send 1 to 6 pictures, 7(17) send video
#define EEPROM_WAKEUPTIME_ADDR (EEPROM_MOTIONEMAILACTION_ADDR+sizeof(motionEmailAction))
int wakeupTime = 11300;  // X div 10000 is send wakeup email (1) or no (0), 
                         // X mod 10000 div 100 is hours, X mod 100 is minutes
#define EEPROM_SIZE_ADDR (EEPROM_WAKEUPTIME_ADDR+sizeof(wakeupTime))

#include "esp_camera.h"
// CAMERA_MODEL_AI_THINKER
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

#include <HTTPClient.h>

unsigned long timeout_10min = 0LL;
int PIRActionActive = 0;  // 0 - not waiting for remote trigger, 1 - waiting for remote trigger
RTC_DATA_ATTR int PIRWakeup = 0;   // 0 - wake from restart or timer trigger, greater than 0 is PIR wakeup


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 

void do_deep_sleep() {  

  Serial.println("Going to deep sleep");

  PIRWakeup = 0;
  
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_X, level)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 0);

  // turn off wifi
  WiFi.disconnect();

  // setup time wakeup
  time(&now);
  localtime_r(&now, &timeinfo);
  char strftime_buf[64];

  strftime(strftime_buf, sizeof(strftime_buf), "%H", &timeinfo);
  int curTH = atoi(strftime_buf); 
  strftime(strftime_buf, sizeof(strftime_buf), "%M", &timeinfo);
  int curTM = atoi(strftime_buf); 
  int curT = (curTH * 60) + curTM;

  int wakeTH = (wakeupTime % 10000) / 100;
  int wakeTM = wakeupTime % 100;
  int wakeT = (wakeTH * 60) + wakeTM;
  if (curT > wakeT) wakeT = wakeT + (24 * 60);
  wakeT = wakeT - curT;
  Serial.println("Scheduled to wakeup at " + String(wakeTH) + " H : " + String(wakeTM) + " M");
  
  esp_sleep_enable_timer_wakeup(wakeT * 60 * uS_TO_S_FACTOR);

  esp_deep_sleep_start();
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 

void do_deep_sleep_1sec() {  

  // turn off wifi
  WiFi.disconnect();

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP1S * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 

void major_fail() {  //flash the led on the ESP32 in case of error, then sleep and reboot

  //for  (int i = 0;  i < 5; i++) {
  for  (int i = 0;  i < 1; i++) {
    digitalWrite(33, LOW);   delay(150);
    digitalWrite(33, HIGH);  delay(150);
    digitalWrite(33, LOW);   delay(150);
    digitalWrite(33, HIGH);  delay(150);
    digitalWrite(33, LOW);   delay(150);
    digitalWrite(33, HIGH);  delay(150);

    delay(150);

    digitalWrite(33, LOW);  delay(500);
    digitalWrite(33, HIGH); delay(150);
    digitalWrite(33, LOW);  delay(500);
    digitalWrite(33, HIGH); delay(150);
    digitalWrite(33, LOW);  delay(500);
    digitalWrite(33, HIGH); delay(150);

    delay(150);

    digitalWrite(33, LOW);   delay(150);
    digitalWrite(33, HIGH);  delay(150);
    digitalWrite(33, LOW);   delay(150);
    digitalWrite(33, HIGH);  delay(150);
    digitalWrite(33, LOW);   delay(150);
    digitalWrite(33, HIGH);  delay(150);

    delay(450);
  }

  // sleep for 1 second and restart
  do_deep_sleep_1sec();
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 

bool init_wifi()
{
  int connAttempts = 0;
  
  // this is the fixed ip stuff 
  IPAddress local_IP(IP_Address);

  // Set your Gateway IP address
  IPAddress gateway(IP_Gateway);

  IPAddress subnet(IP_Subnet);
  IPAddress primaryDNS(IP_PrimaryDNS); // optional
  IPAddress secondaryDNS(IP_SecondaryDNS); // optional

  WiFi.mode(WIFI_STA);

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println(F("STA Failed to configure"));
    major_fail();
  }

  //WiFi.setSleep(false);  // turn off wifi power saving, makes response MUCH faster
  
  WiFi.printDiag(Serial);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED ) {
    delay(500);
    Serial.print(".");
    if (connAttempts > 10) {
      Serial.println(F("Cannot connect"));
      WiFi.printDiag(Serial);
      major_fail();
      return false;
    }
    connAttempts++;
  }
  return true;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 

void init_time() {

  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_setservername(1, "time.windows.com");
  sntp_setservername(2, "time.nist.gov");

  sntp_init();

  // wait for time to be set
  time_t now = 0;
  timeinfo = { 0 };
  int retry = 0;
  const int retry_count = 10;
  while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
    Serial.printf("Waiting for system time to be set... (%d/%d) -- %d\n", retry, retry_count, timeinfo.tm_year);
    delay(2000);
    time(&now);
    localtime_r(&now, &timeinfo);
    Serial.println(ctime(&now));
  }

  if (timeinfo.tm_year < (2016 - 1900)) {
    major_fail();
  }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// GPIO stuff for battery voltage reads
//

#include "esp_adc_cal.h"
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
static esp_adc_cal_characteristics_t *adc_chars;

uint32_t batteryVoltage;    
char batteryMsg[35];  

void Configure_GPIO12() {  // GPIO12 is shared with the SD card - no SD, no worries
  //Configure GPIO12 for ADC to read battery voltage
  adc2_config_channel_atten(ADC2_GPIO12_CHANNEL, ADC_ATTEN_11db);
    
  //Characterize ADC
  adc_chars = (esp_adc_cal_characteristics_t*) calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_11db, ADC_WIDTH_12Bit, DEFAULT_VREF, adc_chars);
}

uint32_t Get_GPIO12_Voltage() {
  // note: when the wifi is on, GPIO12 returns 0. If wifi is needed, turn wifi off, 
  // get the reading, then turn wifi on again.
  
  //Multisampling - average 64 samples to get best value
  uint32_t adc_reading = 0;
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    int read_raw;
    adc2_get_raw(ADC2_GPIO12_CHANNEL, ADC_WIDTH_12Bit, &read_raw);
    adc_reading += read_raw;
  }
  adc_reading /= NO_OF_SAMPLES;

  //Convert adc_reading to voltage in mV
  uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
  //Serial.print("Raw: ");
  //Serial.print(adc_reading);

  // Correct voltage value due to voltage divider circuit
  voltage = (voltage * 147) / 100;
  return voltage;
}


/**********************************************************************************
 * 
 *  From here to end comment below is the contents of sendemail.h
 *
 *  If an SD card is used, it is assumed to be mounted on /sdcard.
 * 
 */

#ifndef __SENDEMAIL_H
#define __SENDEMAIL_H

// uncomment for debug output on Serial port
//#define DEBUG_EMAIL_PORT

// uncomment if using SD card
#define USING_SD_CARD

#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <base64.h>

#ifdef USING_SD_CARD
#include <FS.h>
#include "SD_MMC.h"
#endif


class SendEmail
{
  private:
    const String host;
    const int port;
    const String user;
    const String passwd;
    const int timeout;
    const bool ssl;
    WiFiClient* client;
    String readClient();

    // stuff for attaching buffers (could be images and videos held in memory)
    int attachbuffercount;  // current number of buffer attachments
    static const int attachbuffermaxcount = 10;  // max number of buffers that can be attached
    struct attachbufferitem {
      char * buffername;  // name for buffer
      char * buffer;  // pointer to buffer
      size_t buffersize;  // number of bytes in buffer
    };
    attachbufferitem attachbufferitems[attachbuffermaxcount];
    
#ifdef USING_SD_CARD
    // stuff for attaching files (assumes SD card is mounted as /sdcard)
    int attachfilecount;  // current number of file attachments
    static const int attachfilemaxcount = 10;  // max number of file that can be attached
    struct attachfileitem {
      char * filename;  // name for file
    };
    attachfileitem attachfileitems[attachfilemaxcount];
#endif
    
  public:
    SendEmail(const String& host, const int port, const String& user, const String& passwd, const int timeout, const bool ssl);

    void attachbuffer(char * name, char * bufptr, size_t bufsize);

    void attachfile(char * name);
    
    bool send(const String& from, const String& to, const String& subject, const String& msg);

    void close();
};

#endif

/* end of sendemail.h */


SemaphoreHandle_t baton;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  do_fb - just takes a picture and discards it
//

static esp_err_t do_fb() {
  xSemaphoreTake( baton, portMAX_DELAY );
  camera_fb_t * fb = esp_camera_fb_get();

  Serial.print("Pic, len="); Serial.println(fb->len);

  esp_camera_fb_return(fb);
  xSemaphoreGive( baton );
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 
int cameraRunning = 0;  // 0 - not initialized, 1 - initialized, 2 - error initializing
esp_err_t cam_err = 0;;

static esp_err_t config_camera() {  // returns true if an error, false is no error

  Serial.println(F("config camera"));

  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
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
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  //config.xclk_freq_hz = 20000000;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  //config.fb_count = 20;  // from 12
  //config.fb_count = 12;
  config.fb_count = 4;

  Serial.print(" Before camera init  Free Heap: "); Serial.println(ESP.getFreeHeap());

  // camera init
  esp_camera_deinit();

  cam_err = esp_camera_init(&config);
  if (cam_err != ESP_OK) {
    int k = 1;
    while ( cam_err != ESP_OK && k < 5 ) {
      Serial.printf("Camera init failed with error 0x%x, trying again...", cam_err);
      esp_camera_deinit();
      delay( k * 100 );
      cam_err = esp_camera_init(&config);
      k++;
      if ( cam_err == ESP_OK ) break;
    }
    if (cam_err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", cam_err);
      if ( PIRWakeup > 0 ) {  // retry 10 times 
        if ( PIRWakeup < 10 ) {    
          PIRWakeup++;
          do_deep_sleep_1sec();
        }
        else
          cameraRunning = 2;
      }
      else
        major_fail();
    }
  }

  if (cam_err == ESP_OK) {  // camera init is good
    cameraRunning = 1;
    delay(500);
/*
  sensor_t * ss = esp_camera_sensor_get();
  ss->set_quality(ss, quality);
  ss->set_framesize(ss, (framesize_t)framesize);
  if (gray == 1) {
    ss->set_special_effect(ss, 2);  // 0 regular, 2 grayscale
  } else {
    ss->set_special_effect(ss, 0);  // 0 regular, 2 grayscale
  }
*/
    for (int j = 0; j < 5; j++) {
      do_fb();  // start the camera ... warm it up
      delay(20);
    }
  }
  Serial.print(" After camera init  Free Heap: "); Serial.println(ESP.getFreeHeap());
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//~~~~~~ Start of AVI stuff

// GLOBALS
#define BUFFSIZE 512

// global variable used by these pieces

// don't change the below unless you are certain you know what you are doing
int frames_per_second = 10; 
int capture_interval = round(1000 / frames_per_second); // microseconds between captures 
int total_frames = 10 * frames_per_second;  // frames per second times 10 seconds
int recording = 0;          // turned off until start of setup
int framesize = 6;          // 10 uxga, 7 SVGA, 6 VGA, 5 CIF
int repeat = 0;             // no repeating videos at this time
int quality = 10;           // 10 on the 0..64 scale, lower number is better quality
int xspeed = 1;             // playback speed multiplier
int gray = 0;               // 1 grayscale, 0 color
int xlength = total_frames * capture_interval / 1000;

char aviname[100];

uint8_t buf[BUFFSIZE];

unsigned long current_millis;
unsigned long last_capture_millis = 0;

uint8_t * bufpos = 0;
uint8_t * idxpos = 0;
uint16_t frame_cnt = 0;
uint16_t remnant = 0;
uint32_t startms;
uint32_t elapsedms;
uint32_t uVideoLen = 0;
int other_cpu_active = 0;
int skipping = 0;
int skipped = 0;

int newfile = 0;
int frames_so_far = 0;
unsigned long bp;
unsigned long bw;
unsigned long totalp;
unsigned long totalw;

// will eventually allocate 850 * 4096 (3,481,600) byte buffer for 10 second video in frameb
  // allocate 850 * 4096 (3,481,600) byte buffer for 10 second video
  // allocate 950 * 4096 (3,891,200) byte buffer for 10 second video
  // allocate 925 * 4096 (3,788,800) byte buffer for 10 second video
  // allocate 900 * 4096 (3,686,400) byte buffer for 10 second video

uint8_t * frameb = NULL;
size_t frameb_size = 923 * 4096;
size_t frameb_len = 0;
uint8_t * indexb = NULL;

//int fb_max = 12;
int fb_max = 4;

camera_fb_t * fb_q[30];
int fb_in = 0;
int fb_out = 0;

camera_fb_t * fb = NULL;

#define AVIOFFSET 240 // AVI main header length

unsigned long movi_size = 0;
unsigned long jpeg_size = 0;
unsigned long idx_offset = 0;

const uint8_t zero_buf[4] = {0x00, 0x00, 0x00, 0x00};
const uint8_t   dc_buf[4] = {0x30, 0x30, 0x64, 0x63};    // "00dc"
const uint8_t avi1_buf[4] = {0x41, 0x56, 0x49, 0x31};    // "AVI1"
const uint8_t idx1_buf[4] = {0x69, 0x64, 0x78, 0x31};    // "idx1"

const uint8_t  vga_w[2] = {0x80, 0x02}; // 640
const uint8_t  vga_h[2] = {0xE0, 0x01}; // 480
//const uint8_t  cif_w[2] = {0x90, 0x01}; // 400
//const uint8_t  cif_h[2] = {0x28, 0x01}; // 296
//const uint8_t svga_w[2] = {0x20, 0x03}; // 800
//const uint8_t svga_h[2] = {0x58, 0x02}; // 600
//const uint8_t uxga_w[2] = {0x40, 0x06}; // 1600
//const uint8_t uxga_h[2] = {0xB0, 0x04}; // 1200


const int avi_header[AVIOFFSET] PROGMEM = {
  0x52, 0x49, 0x46, 0x46, 0xD8, 0x01, 0x0E, 0x00, 0x41, 0x56, 0x49, 0x20, 0x4C, 0x49, 0x53, 0x54,
  0xD0, 0x00, 0x00, 0x00, 0x68, 0x64, 0x72, 0x6C, 0x61, 0x76, 0x69, 0x68, 0x38, 0x00, 0x00, 0x00,
  0xA0, 0x86, 0x01, 0x00, 0x80, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
  0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54, 0x84, 0x00, 0x00, 0x00,
  0x73, 0x74, 0x72, 0x6C, 0x73, 0x74, 0x72, 0x68, 0x30, 0x00, 0x00, 0x00, 0x76, 0x69, 0x64, 0x73,
  0x4D, 0x4A, 0x50, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x74, 0x72, 0x66,
  0x28, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00,
  0x01, 0x00, 0x18, 0x00, 0x4D, 0x4A, 0x50, 0x47, 0x00, 0x84, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x4E, 0x46, 0x4F,
  0x10, 0x00, 0x00, 0x00, 0x6A, 0x61, 0x6D, 0x65, 0x73, 0x7A, 0x61, 0x68, 0x61, 0x72, 0x79, 0x20,
  0x76, 0x33, 0x39, 0x20, 0x4C, 0x49, 0x53, 0x54, 0x00, 0x01, 0x0E, 0x00, 0x6D, 0x6F, 0x76, 0x69,
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// AviWriterTask runs on cpu 1 to write the avi file
//

TaskHandle_t CameraTask, AviWriterTask;

void codeForAviWriterTask( void * parameter )
{

  for (;;) {
    make_avi_buf();
    delay(1);
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// CameraTask runs on cpu 0 to take pictures and drop them in a queue
//

void codeForCameraTask( void * parameter )
{

  for (;;) {

    if (other_cpu_active == 1 ) {
      current_millis = millis();
      if (current_millis - last_capture_millis > capture_interval) {

        last_capture_millis = millis();

        xSemaphoreTake( baton, portMAX_DELAY );

        if  ( ( (fb_in + fb_max - fb_out) % fb_max) + 1 == fb_max ) {
          xSemaphoreGive( baton );

          Serial.print(" S ");  // the queue is full
          skipped++;
          skipping = 1;

        } 
        if (skipping > 0 ) {

          if (skipping % 2 == 0) {  // skip every other frame until queue is cleared

            frames_so_far = frames_so_far + 1;
            frame_cnt++;

            fb_in = (fb_in + 1) % fb_max;
            bp = millis();
            fb_q[fb_in] = esp_camera_fb_get();
            totalp = totalp - bp + millis();

          } else {
            //Serial.print(((fb_in + fb_max - fb_out) % fb_max));  Serial.print("-s ");  // skip an extra frame to empty the queue
            skipped++;
          }
          skipping = skipping + 1;
          if (((fb_in + fb_max - fb_out) % fb_max) == 0 ) {
            skipping = 0;
            Serial.print(" == ");
          }

          xSemaphoreGive( baton );

        } else {

          skipping = 0;
          frames_so_far = frames_so_far + 1;
          frame_cnt++;

          fb_in = (fb_in + 1) % fb_max;
          bp = millis();
          fb_q[fb_in] = esp_camera_fb_get();
          totalp = totalp - bp + millis();
          xSemaphoreGive( baton );

        }
      } else {
        //delay(5);     // waiting to take next picture
      }
    } else {
      //delay(50);  // big delay if not recording
    }
    delay(1);
  }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Writes an uint32_t in Big Endian at current buffer position
//
static void inline print_quartet(unsigned long i, uint8_t * bptr)
{
  uint8_t x[1];

  x[0] = i % 0x100;
  memcpy( bptr, x, 1 );
  //size_t i1_err = fwrite(x , 1, 1, fd);
  i = i >> 8;  x[0] = i % 0x100;
  memcpy( bptr+1, x, 1 );
  //size_t i2_err = fwrite(x , 1, 1, fd);
  i = i >> 8;  x[0] = i % 0x100;
  memcpy( bptr+2, x, 1 );
  //size_t i3_err = fwrite(x , 1, 1, fd);
  i = i >> 8;  x[0] = i % 0x100;
  memcpy( bptr+3, x, 1 );
  //size_t i4_err = fwrite(x , 1, 1, fd);
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Make the avi movie in 4 pieces
//
//   make_avi_buf() called in every loop, which calls below, depending on conditions
//   start_avi_buf() - open the file and write headers
//   another_pic_avi_avi() - write one more frame of movie
//   end_avi_buf() - write the final parameters and close the file

void make_avi_buf() {

  // we are recording, but no file is open

  if (newfile == 0 && recording == 1) {                                     // open the file

    digitalWrite(33, HIGH);
    newfile = 1;
    start_avi_buf();

  } else {

    // we have a file open, but not recording

    if (newfile == 1 && recording == 0) {                                  // got command to close file

      digitalWrite(33, LOW);
      end_avi_buf();

      Serial.println(F("Done capture due to command"));

      frames_so_far = total_frames;

      newfile = 0;    // file is closed
      recording = 0;  // DO NOT start another recording

    } else {

      if (newfile == 1 && recording == 1) {                            // regular recording

        if (frames_so_far >= total_frames || (movi_size + ((fb_max + 1) * 64000)) > frameb_size )  {   // we are done the recording

          Serial.println(F("Done capture for total frames!"));

          digitalWrite(33, LOW);                                                       // close the file
          end_avi_buf();

          frames_so_far = 0;
          newfile = 0;          // file is closed

          if (repeat > 0) {
            recording = 1;        // start another recording
            repeat = repeat - 1;
          } else {
            recording = 0;
          }

        } else if ((millis() - startms) > (total_frames * capture_interval)) {

          Serial.println (" "); Serial.println("Done capture for time");
          Serial.print("Time Elapsed: "); Serial.print(millis() - startms); Serial.print(" Frames: "); Serial.println(frame_cnt);
          Serial.print("Config:       "); Serial.print(total_frames * capture_interval ) ; Serial.print(" (");
          Serial.print(total_frames); Serial.print(" x "); Serial.print(capture_interval);  Serial.println(")");

          digitalWrite(33, LOW);                                                       // close the file

          end_avi_buf();

          frames_so_far = 0;
          newfile = 0;          // file is closed
          if (repeat > 0) {
            recording = 1;        // start another recording
            repeat = repeat - 1;
          } else {
            recording = 0;
          }

        } else  {                                                            // regular

          another_save_avi_buf();

        }
      }
    }
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// start_avi - open the files and write in headers
//

static esp_err_t start_avi_buf() {

  Serial.println(F("Starting an avi "));

  for (int j = 0; j < 5; j++) {
    do_fb();  // start the camera ... warm it up
    delay(20);
  }

  time(&now);
  localtime_r(&now, &timeinfo);
  char strftime_buf[64];

  strftime(strftime_buf, sizeof(strftime_buf), "%F_%H_%M_%S", &timeinfo);
  sprintf(aviname, "%s_VGA_L0_M0.avi", strftime_buf);
  Serial.print("\nFile name will be (size added at end)>"); Serial.print(aviname); Serial.println("<");

  frameb = (uint8_t*) heap_caps_calloc( frameb_size, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  // allocate buffer for frame index 
  indexb = (uint8_t*) heap_caps_calloc( (total_frames + 10) * 8, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  // load AVI header into video buffer
  for ( int i = 0; i < AVIOFFSET; i++)
  {
    char ch = pgm_read_byte(&avi_header[i]);
    buf[i] = ch;
  }
  memcpy( frameb, buf, AVIOFFSET );
  bufpos = frameb + AVIOFFSET;

  // write vga info
  memcpy( frameb + 0x40, vga_w, 2 );
  memcpy( frameb + 0xA8, vga_w, 2 );
  memcpy( frameb + 0x44, vga_h, 2 );
  memcpy( frameb + 0xAC, vga_h, 2 );
  
  Serial.print(F("\nRecording "));
  Serial.print(total_frames);
  Serial.println(F(" video frames ...\n"));

  startms = millis();
  totalp = 0;
  totalw = 0;
  jpeg_size = 0;
  movi_size = 0;
  uVideoLen = 0;
  idx_offset = 4;
  idxpos = indexb;

  frame_cnt = 0;
  frames_so_far = 0;

  skipping = 0;
  skipped = 0;

  newfile = 1;

  other_cpu_active = 1;

} // end of start avi

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  another_save_avi runs on cpu 1, saves another frame to the avi file
//
//  the "baton" semaphore makes sure that only one cpu is using the camera subsystem at a time
//

static esp_err_t another_save_avi_buf() {

  xSemaphoreTake( baton, portMAX_DELAY );

  if (fb_in == fb_out) {        // nothing to do

    xSemaphoreGive( baton );

  } else {

    fb_out = (fb_out + 1) % fb_max;

    int fblen;
    fblen = fb_q[fb_out]->len;

    digitalWrite(33, LOW);

    jpeg_size = fblen;
    movi_size += jpeg_size;
    uVideoLen += jpeg_size;

    bw = millis();
    // write start of frame
    memcpy( bufpos, dc_buf, 4 );
    bufpos = bufpos + 4;
    memcpy( bufpos, zero_buf, 4 );
    bufpos = bufpos + 4;

    // write frame buffer
    memcpy( bufpos, fb_q[fb_out]->buf, fb_q[fb_out]->len );
    bufpos = bufpos + jpeg_size;
    
    esp_camera_fb_return(fb_q[fb_out]);     // release that buffer back to the camera system
    xSemaphoreGive( baton );

    remnant = (4 - (jpeg_size & 0x00000003)) & 0x00000003;

    // save frame index info
    print_quartet(idx_offset, idxpos);
    idxpos = idxpos + 4;
    print_quartet(jpeg_size, idxpos);
    idxpos = idxpos + 4;

    idx_offset = idx_offset + jpeg_size + remnant + 8;

    jpeg_size = jpeg_size + remnant;
    movi_size = movi_size + remnant;
    if (remnant > 0) {
      memcpy( bufpos, zero_buf, remnant );
    }
    bufpos = bufpos + remnant;  // Here, we are at end of chunk (after padding)

    print_quartet( jpeg_size, bufpos - jpeg_size - 4 );  // Overwrite placeholder with actual frame size (without padding)

    // Overwrite "JFIF" (still images) with more appropriate "AVI1"
    memcpy( bufpos - jpeg_size + 6, avi1_buf, 4 );

    totalw = totalw + millis() - bw;

    digitalWrite(33, HIGH);
  }

} // end of another_pic_avi


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  end_avi runs on cpu 1, empties the queue of frames, writes the index, and closes the files
//

static esp_err_t end_avi_buf() {

  unsigned long current_end = 0;

  other_cpu_active = 0 ;

  Serial.print(" Write Q: "); Serial.print((fb_in + fb_max - fb_out) % fb_max); Serial.print(" in/out  "); Serial.print(fb_in); Serial.print(" / "); Serial.println(fb_out);

  for (int i = 0; i < fb_max; i++) {           // clear the queue
    another_save_avi_buf();
  }

  Serial.print(" Write Q: "); Serial.print((fb_in + fb_max - fb_out) % fb_max); Serial.print(" in/out  "); Serial.print(fb_in); Serial.print(" / "); Serial.println(fb_out);

  Serial.println(F("End of avi - closing the files"));

  elapsedms = millis() - startms;
  float fRealFPS = (1000.0f * (float)frame_cnt) / ((float)elapsedms) * xspeed;
  float fmicroseconds_per_frame = 1000000.0f / fRealFPS;
  uint8_t iAttainedFPS = round(fRealFPS);
  uint32_t us_per_frame = round(fmicroseconds_per_frame);

  //Modify the MJPEG header from the beginning of the file, overwriting various placeholders

  print_quartet( movi_size + 240 + 16 * frame_cnt + 8 * frame_cnt, frameb + 4 );

  print_quartet( us_per_frame, frameb + 0x20 );

  unsigned long max_bytes_per_sec = movi_size * iAttainedFPS / frame_cnt;

  print_quartet( max_bytes_per_sec, frameb + 0x24 );

  print_quartet( frame_cnt, frameb + 0x30 );

  print_quartet( frame_cnt, frameb + 0x8c );

  print_quartet( (int)iAttainedFPS, frameb + 0x84 );

  print_quartet( movi_size + frame_cnt * 8 + 4, frameb + 0xe8 );

  Serial.println(F("\n*** Video recorded and saved ***\n"));
  Serial.print(F("Recorded "));
  Serial.print((elapsedms + 500) / 1000);
  Serial.print(F("s in "));
  Serial.print(frame_cnt);
  Serial.print(F(" frames\nFile size is "));
  Serial.print(movi_size + 12 * frame_cnt + 4);
  Serial.print(F(" bytes\nActual FPS is "));
  Serial.print(fRealFPS, 2);
  Serial.print(F("\nMax data rate is "));
  Serial.print(max_bytes_per_sec);
  Serial.print(F(" byte/s\nFrame duration is "));  Serial.print(us_per_frame);  Serial.println(F(" us"));
  Serial.print(F("Average frame length is "));  Serial.print(uVideoLen / frame_cnt);  Serial.println(F(" bytes"));
  Serial.print(F("Average picture time (ms) ")); Serial.println( totalp / frame_cnt );
  Serial.print(F("Average write time (ms) ")); Serial.println( totalw / frame_cnt );
  Serial.print(F("Frames Skipped % "));  Serial.println( 100.0 * skipped / frame_cnt, 1 );

  Serial.println(F("Writing the index"));
  memcpy( bufpos, idx1_buf, 4 );
  bufpos = bufpos + 4;

  print_quartet(frame_cnt * 16, bufpos);
  bufpos = bufpos + 4;
  
  char * AteBytes;
  AteBytes = (char*) malloc (8);
  idxpos = indexb;
  
  // save index of frame positions into AVI
  for (int i = 0; i < frame_cnt; i++) {
    memcpy( bufpos, dc_buf, 4 );
    bufpos = bufpos + 4;
    memcpy( bufpos, zero_buf, 4 );
    bufpos = bufpos + 4;
    memcpy( bufpos, idxpos, 8 );
    bufpos = bufpos + 8;
    idxpos = idxpos + 8;
  }
  free( AteBytes );
  free( indexb );

  frameb_len = bufpos - frameb;
  
  // add size to end of avi file name
  int avisz = (frameb_len / 1000000) + 1;
  if ( frameb_len == 0 ) { avisz = 0; }

  uint32_t length = (uint32_t) ((elapsedms + 500) / 1000);

  aviname[strlen(aviname)-8] = '\0';

  char newname[100];
  sprintf(newname, "%s%i_M%i.avi", aviname, length, avisz);   
  strcpy( aviname, newname );
  
  Serial.print(F("\nFinal file name is> ")); Serial.print(aviname); Serial.println(" <");
  // the M in the filename now indicates size in Megabytes

  Serial.println("---");

}
//~~~~~~ End of AVI stuff


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 
// 
void startCamera() {
  if ( cameraRunning == 0 ) {
    
    // temporary grab memory to keep camera from taking too much
    static char * memtmp = (char *) malloc(32767);
    static char * memtmp2 = (char *) malloc(32767);
  
    config_camera();

    // give the memory back
    free(memtmp2);
    memtmp2 = NULL;
    free(memtmp);
    memtmp = NULL;
  }
}


int emailvideostatus = 0;  // 0 - not running, 1 - running, 2 - ended good, 3 - ended bad

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 
// 
bool email_video( char * subject, char * message ) {

  // make a 10 second video stored in a buffer, then email it

  // start camera if not already started
  startCamera();

  if ( cameraRunning == 2 ) {  // camera init failed
    char errnum[40];
    sprintf( errnum, "\n\n Camera failed - error 0x%x", cam_err );
    strcat( message, errnum );  
  } else {

    // aviname will be set in start_avi_buf
    // frameb will be allocated in start_avi_buf
    // frameb will contain the finished avi file when the recording stops
    // fraemb_len will contain the avi file length when the recording stops
    // frameb will need to be freed after the video email is sent

    // start avi recording (calls start_avi_buf)
    recording = 1;

    // wait for avi recording to end
    while ( recording == 1 ) { delay(100); }
  } 

  while ( PIRActionActive == 1 ) {  // wait for remote PIR action to finish before sending email
    delay(100);
  }
   
  Serial.print(F(" in emailvideo  Heap is: ")); Serial.println(ESP.getFreeHeap());
  
  // create SendEmail object 
  SendEmail e(emailhost, emailport, emailsendaddr, emailsendpwd, 5000, true); 
  delay(100);

  if ( cameraRunning == 1 ) {  // attach video
    e.attachbuffer( aviname, (char *)frameb, frameb_len );
  }
  
  // Send Email
  char send[40];
  sprintf(send,"\<%s\>", emailsendaddr);
  char recv[40];
  sprintf(recv,"\<%s\>", email);
  //Serial.println(F("SendEmail object created, now trying to send..."));

  int j = 1;  // used to loop five times trying to send email
  bool result = e.send( send, recv, subject, message ); 
  while ( !result && j < 5 ) {  // retry sending email 4 more times
    delay( j * 1000 );
    result = e.send( send, recv, subject, message ); 
    j++;   
  }
  if ( result ) { 
    Serial.println(F("Email sent")); 
    emailvideostatus = 2; 
  } else { 
    Serial.println(F("Failed to send email")); 
    emailvideostatus = 3; 
  }

  e.close(); // close email client

  if ( cameraRunning == 1 ) {  
    free( frameb );
    frameb = NULL;
  }
  
  return result;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 
// 
bool email_pictures( char * subject, char * message, int pictureCount ) {
  // there is a one second delay between pictures
  // returns true if good, false if failed

  // start camera if not already started
  startCamera();

  // add up to ten buffer attachments 
  uint8_t * framebp[10];
  size_t frameblen[10];
  char fname[10][100];

  if ( cameraRunning == 2 ) {  // camera init failed
    char errnum[40];
    sprintf( errnum, "\n\n Camera failed - error 0x%x", cam_err );
    strcat( message, errnum );  
  } else {

    for ( int i = 0; i < pictureCount; i++ ) {
      // build name for new jpg picture
      time(&now);
      localtime_r(&now, &timeinfo);
      char strftime_buf[64];
      strftime(strftime_buf, sizeof(strftime_buf), "%F_%H_%M_%S", &timeinfo);

      sprintf(fname[i], "%s.jpg", strftime_buf);
      Serial.print(F("Email picture name is > ")); Serial.print(fname[i]); Serial.println(" <");

      // get a camera frame to save as jpg
      camera_fb_t * fb = NULL;
      xSemaphoreTake( baton, portMAX_DELAY );
      fb = esp_camera_fb_get();

      if (!fb) {
        Serial.println(F("Camera capture failed"));
        xSemaphoreGive( baton );
        return false;
      }

      // allocate memory for frame buffer from PSRAM
      frameblen[i] = fb->len;
      framebp[i] = (uint8_t *) heap_caps_calloc(frameblen[i], 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      // put current fb in it
      memcpy(framebp[i], fb->buf, frameblen[i]);
      esp_camera_fb_return(fb);
      xSemaphoreGive( baton );

      delay(1000);  // one second between pictures
    }
  }
  
  while ( PIRActionActive == 1 ) {  // wait for remote PIR action to finish before sending email
    delay(100);
  }
   
  Serial.print(F(" waiting in emailpicture  Heap is: ")); Serial.println(ESP.getFreeHeap());
  
  // create SendEmail object 
  SendEmail e(emailhost, emailport, emailsendaddr, emailsendpwd, 5000, true); 
  delay(100);

  if ( cameraRunning == 1 ) {  // attach pictures
    for ( int i = 0; i < pictureCount; i++ ) {
      e.attachbuffer( fname[i], (char *)framebp[i], frameblen[i] );
    }
  }  

  // Send Email
  char send[40];
  sprintf(send,"\<%s\>", emailsendaddr);
  char recv[40];
  sprintf(recv,"\<%s\>", email);
  //Serial.println(F("SendEmail object created, now trying to send..."));

  int j = 1;  // used to loop five times trying to send email
  bool result = e.send( send, recv, subject, message ); 
  while ( !result && j < 5 ) {  // retry sending email 4 more times
    delay( j * 1000 );
    result = e.send( send, recv, subject, message ); 
    j++;   
  }
  if ( result ) { Serial.println(F("Email sent")); }
  else { Serial.println(F("Failed to send email")); }

  e.close(); // close email client
  if ( cameraRunning == 1 ) {  
    for ( int i = 0; i < pictureCount; i++ ) {
     free( framebp[i] );
     framebp[i] = NULL;
    }
  }
  
  return result;
}


char * the_page = NULL;  // used to hold complete response page before its sent to browser

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 
// 
static esp_err_t refresh_picture(httpd_req_t *req) {

  // build name for new jpg picture
  time(&now);
  localtime_r(&now, &timeinfo);

  char strftime_buf[64];
  strftime(strftime_buf, sizeof(strftime_buf), "%F_%H_%M_%S", &timeinfo);

  char fname[100];

  sprintf(fname, "%s.jpg", strftime_buf);
  Serial.print(F("\nPicture name is > ")); Serial.print(fname); Serial.println(" <");

  // get a camera frame to save as jpg
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  xSemaphoreTake( baton, portMAX_DELAY );
  fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println(F("Camera capture failed"));
    httpd_resp_send_500(req);
    xSemaphoreGive( baton );
    return ESP_FAIL;
  }

  // now send the new jpg back to the browser
  char buf[200];
  sprintf(buf, "inline; filename=%s", fname);

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", buf);

  httpd_resp_send(req, (const char *)fb->buf, fb->len);

  esp_camera_fb_return(fb);
  xSemaphoreGive( baton );

  return ESP_OK;
}


// definitions for web page header stream content
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 
static esp_err_t stream_handler(httpd_req_t *req) {

  timeout_10min = millis();

  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  char * part_buf[64];

  // wait one second
  delay(100);

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while ( true ) {
    xSemaphoreTake( baton, portMAX_DELAY );
    fb = esp_camera_fb_get();

    if (!fb) {
      Serial.println(F("Camera capture failed"));
      httpd_resp_send_500(req);
      xSemaphoreGive( baton );
      return ESP_FAIL;
    }

    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, fb->len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *) fb->buf, fb->len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    else {
      res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    }

    esp_camera_fb_return(fb);
    fb = NULL;
    xSemaphoreGive( baton );

    if(res != ESP_OK){
      Serial.println( F("Breaking out of stream loop due to error response from browser") );
      delay(1000); 
      break;
    }

    timeout_10min = millis();
  }

  return ESP_OK;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 
void do_index() {

  Serial.println("do_index ");

  const char msg[] PROGMEM = R"rawliteral(<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CAM Remote PIR</title>
<style> 
#image-container {
  width: 640px;
  height: 480px;
  background-color: powderblue;
}
a {color: blue;}
a:visited {color: blue;}
a:hover {color: blue;}
a:active {color: blue;}
</style>
<script>

  var streaming = 0;
  var videoactive = 0;
  
  document.addEventListener('DOMContentLoaded', function() {
    streaming = 0;
    do_picture();
    setTimeout(function(){ window.scrollTo(0,0); }, 1000);
  });

  function do_picture() {
    if ( streaming == 1 )
      alert( "Disabled while streaming" );
    else if ( videoactive == 1 )
      alert( "Disabled while emailing 10 second video" );
    else {
      var c = document.location.origin;
      var xhr = new XMLHttpRequest();
      xhr.open('GET', `${c}/?action=picture&_cb=${Date.now()}`, true);
      xhr.overrideMimeType('text/plain; charset=x-user-defined');
      xhr.onloadend = function(){
        var binary = "";
        for(i=0;i<xhr.response.length;i++){
          binary += String.fromCharCode(xhr.response.charCodeAt(i) & 0xff);
        }
        // the next four lines get the filename
        var cd = xhr.getResponseHeader('Content-Disposition');
        var startIndex = cd.indexOf("filename=") + 9; 
        var endIndex = cd.length; 
        var fname = cd.substring(startIndex, endIndex);
        // the next section wraps the image in a link so the filename is known for downloading
        document.getElementById('image-container').innerHTML = 
          '<a href="data:image/jpeg;base64,' + btoa(binary) + `" download="${fname}">
          <img id="pic" src=""></a>`;
        document.getElementById('pic').src = 'data:image/jpeg;base64,' + btoa(binary);
      }
      xhr.send(null);  
    }
  }

  function do_stream() {
    var c = document.location.origin;
    
    // toggle streaming
    if ( videoactive == 1 )
      alert( "Disabled while emailing 10 second video" );
    else {
      if ( streaming == 0 ) {
        streaming = 1;
        document.getElementById('stream').innerHTML = "Stop Streaming";
        document.getElementById('image-container').innerHTML = 
          '<img id="strimg" src="'+`${c}/stream?_cb=${Date.now()}`+'">';
      } else {
        streaming = 0;
        document.getElementById('stream').innerHTML = "Start Streaming";
        document.getElementById('strimg').setAttribute("src","");
        document.getElementById('strimg').src="";
        document.getElementById('image-container').removeChild(strimg);
        document.getElementById('image-container').innerHTML = "";
        document.getElementById('image-container').textContent="";
        do_picture();
      }
    }
  }

  var pictureactive = 0;
 
  function do_emailpicture() {
    if ( streaming == 1 )
      alert( "Disabled while streaming" );
    else if ( videoactive == 1 )
      alert( "Disabled while emailing 10 second video" );
    else {
      if ( pictureactive == 1 ) alert( "Already emailing a picture" );
      else {
        pictureactive = 1;
        var XHR = new XMLHttpRequest();
        XHR.open( "GET", `/?action=emailpicture`, true );  
        XHR.onloadend = function(){
          alert( XHR.responseText );
          pictureactive = 0;
        }
        XHR.send( null );
      }
    }
  }

  function do_emailvideostatus() {
    var XHR = new XMLHttpRequest();
    XHR.open( "GET", `/?action=emailvideostatus`, true );  
    XHR.onloadend = function(){
      if ( XHR.responseText < 2 ) { 
        if ( XHR.responseText == 1 ) {
          setTimeout(function(){ do_emailvideostatus(); }, 10000); 
        }
      } else { 
        if ( XHR.responseText == 2 )
          alert( "Sent an email with a video" );
        else
          alert( "Sending an email with a video failed" );
        videoactive = 0;
      }
    }
    XHR.send( null );
  }  

  function do_emailvideo() {
    if ( streaming == 1 )
      alert( "Disabled while streaming" );
    else {
      if ( videoactive == 1 ) alert( "Already emailing a video\nWait for it to finish" );
      else {
        videoactive = 1;
        var XHR = new XMLHttpRequest();
        XHR.open( "GET", `/?action=emailvideo`, true );  
        XHR.send( null );
        setTimeout(function(){ do_emailvideostatus(); }, 10000);
        alert( "A 10 second video recording has started\nThe email part will take a while\nWait for it to finish" );
      }  
    }
  }

  function do_settings() {
    if ( videoactive == 1 )
      alert( "Disabled while emailing 10 second video" );
    else
      window.location.href = `/settings`;
  }
  
  // force reload when going back to page
  if (!!window.performance && window.performance.navigation.type == 2) {
    window.location.reload();
  }
  
</script>
</head>
<body><center>
<h1>ESP32-CAM Remote PIR</h1>
The battery voltage is %dmV<br><br>
<div id="image-container"></div>
<br><table><tr>
<td><a id="pic" href="javascript:do_picture();">Refresh Picture</a></td>
<td>&nbsp&nbsp&nbsp</td> 
<td><a id="stream" href="javascript:do_stream();">Start Streaming</a></td>
<td>&nbsp&nbsp&nbsp</td> 
<td><a id="empic" href="javascript:do_emailpicture();">Email a Picture</a></td> 
<td>&nbsp&nbsp&nbsp</td>
<td><a id="emvid" href="javascript:do_emailvideo();">Email a 10 Second Video</a></td> 
<td>&nbsp&nbsp&nbsp</td>
<td><a id="settings" href="javascript:do_settings();">Settings</a></td>
</tr></table><br>

</center></body>
</html>)rawliteral";

  sprintf(the_page, msg, batteryVoltage);
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 
static esp_err_t index_handler(httpd_req_t *req) {

  //Serial.print(F("in index_handler, Heap is: ")); Serial.println(ESP.getFreeHeap());

  timeout_10min = millis();

  char buf[500];
  size_t buf_len;
  char param[60];
  char action[20];

  strcpy(action, "show");

  // query parameters - get action
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "action", param, sizeof(param)) == ESP_OK) {
        strcpy(action, param);
      }
    }
  }

  //Serial.print(F("index_hander with action: ")); Serial.println( action );

  if ( !strcmp( action, "picture" ) ) {
    return refresh_picture(req);
    
  } else if ( !strcmp( action, "emailpicture" ) ) {
    static char sub[30];
    strcpy( sub, "Camera Event" );
    static char msg[120];
    strcpy( msg, "Manual trigger, please see attached picture\n" );
    strcat( msg, batteryMsg );

    if ( email_pictures( sub, msg, 1 ) ) {
      sprintf(the_page, "Sent an email with a picture");
    } else {
      sprintf(the_page, "Sending an email with a picture failed");    
    }
    
    Serial.println( the_page );
    httpd_resp_send(req, the_page, strlen(the_page));

  } else if ( !strcmp( action, "emailvideo" ) ) {

    if ( emailvideostatus == 1 ) {
      sprintf(the_page, "Email 10 second video already running");    
    } else {
      emailvideostatus = 1;
      sprintf(the_page, "Starting email 10 second video");    
    }
    
    Serial.println( the_page );
    httpd_resp_send(req, the_page, strlen(the_page));

  } else if ( !strcmp( action, "emailvideostatus" ) ) {

    sprintf(the_page, "%d", emailvideostatus);
    Serial.println( the_page );
    httpd_resp_send(req, the_page, strlen(the_page));

  } else {
    // display index page
    startCamera();  // only runs once (if cammeraRunning is 0)
    do_index();
    httpd_resp_send(req, the_page, strlen(the_page));
  }
  
  return ESP_OK;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 
String send_http_url(char * url) {

  String response;
  HTTPClient http;   // create an http client object
  http.begin(url);   // setup the url to send 
  http.setTimeout(25000);
  int httpCode = http.GET();   // send the url and get the return code
    if (httpCode == HTTP_CODE_OK) {
      response = http.getString();   // read response
    } else {
      response = http.errorToString(httpCode);
    }
  http.end();
  return response;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 
void do_reset() {

  Serial.println("do_reset ");

  const char msg[] PROGMEM = R"rawliteral(<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CAM Remote PIR</title>
<script>
  document.addEventListener('DOMContentLoaded', function() {
    var c = document.location.origin;
    var i = 15;
    var timing = 1000;
    function loop() {
      document.getElementById('image-container').innerText = 'ESP32-CAM will be ready in '+`${i}`+' seconds';
      i = i - 1;
      if ( i > 0 ) {
        window.setTimeout(loop, timing);
      } else {
        window.location.replace("/");
      }
    }
    loop();
  });
</script>
</head>
<body><center>
<h1>ESP32-CAM Resetting</h1>
<h2><div id="image-container"></div></h2>
If the ESP32-CAM does not respond after 15 seconds, keep waiting.<br>
It will keep resetting about once every 15 seconds if it does not start clean.
</center></body>
</html>)rawliteral";

  sprintf(the_page,  msg);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 
static esp_err_t reset_handler(httpd_req_t *req) {

  do_reset();
  httpd_resp_send(req, the_page, strlen(the_page));

  delay(3000);  // wait 3 seconds
  
  // sleep 1 second and boot up
  do_deep_sleep_1sec();
  
  return ESP_OK;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 
void do_settings(httpd_req_t *req) {

  Serial.println(F("do_settings ")); 

  const char msg1[] PROGMEM = R"rawliteral(<!doctype html>
<html>
<head>
<meta http-equiv="Content-Type" content="text/html; charset=UTF-8" />
<title>ESP32-CAM Remote PIR</title>
<style>
a {color: blue;}
a:visited {color: blue;}
a:hover {color: blue;}
a:active {color: blue;}
</style>
<script>

  document.addEventListener('DOMContentLoaded', function() {
    do_remote(0);
    do_remoteaction(0);
    do_piremail(0);
    do_piraction(0);
    do_wakeupaction(0);
    do_time(0);
  });

function do_reset() {
  if ( confirm( 'Are you sure you want to reboot the ESP32-CAM?' ) ) {
    window.location = "/settings?action=reset";
  }  
}

var timeout = null;

var curremote = %i;

function do_remote(action) {
  if ( action == 0 ) { // display current value
    if ( curremote < 10 ) { // remote trigger is off
      document.getElementById('remote').innerHTML = '<a href="javascript:do_remote(1);">Turn On Remote Trigger</a>';
    } else { // remote trigger is on
      document.getElementById('remote').innerHTML = '<a href="javascript:do_remote(1);">Turn Off Remote Trigger</a>';    
    }
  } else { // toggle value
    if ( curremote < 10 ) { // turn on remote trigger
      curremote = curremote + 10;
      document.getElementById('remote').innerHTML = '<a href="javascript:do_remote(1);">Turn Off Remote Trigger</a>';
    } else { // turn off remote trigger
      curremote = curremote - 10;
      document.getElementById('remote').innerHTML = '<a href="javascript:do_remote(1);">Turn On Remote Trigger</a>';    
    }
    var XHR = new XMLHttpRequest();
    XHR.open( "GET", `/settings?action=remote&raction=${curremote}`, true );   
    XHR.onloadend = function(){
      alert( XHR.responseText );
    }
    XHR.send( null );
  }
}

function do_remoteip() {
  var remip = document.getElementById("remoteip").value + "";
  var XHR = new XMLHttpRequest();
  XHR.open( "GET", `/settings?action=remoteip&remoteipaddr=${remip}`, true );  
  XHR.onloadend = function(){
    alert( XHR.responseText );
  }
  XHR.send( null );
}

function do_remoteaction(action) {
  var a = curremote; // a is action
  if ( a > 9 ) 
    a = a - 10;
  var of = 0; // of is on off value
  if ( curremote > 9 ) 
    of = 10;
  if ( action == 0 ) { // display current value
    document.getElementById("remoteaction").selectedIndex = a;
  } else { // process a possible change
    var e = document.getElementById("remoteaction");
    var na = parseInt(e.options[e.selectedIndex].value) - 1;
    if ( na != a ) { // drop down selection has changed
      curremote = of + na;
      var XHR = new XMLHttpRequest();
      XHR.open( "GET", `/settings?action=remote&raction=${curremote}`, true );   
      XHR.onloadend = function(){
        alert( XHR.responseText );
      }
      XHR.send( null );
    }
  }
}
        
function do_remotetest() {
  var XHR = new XMLHttpRequest();
  XHR.open( "GET", `/settings?action=remotetest`, true );  
  XHR.onloadend = function(){
    alert( XHR.responseText );
  }
  XHR.send( null );
}

function do_email() {
  var email = document.getElementById("email").value;
  var XHR = new XMLHttpRequest();
  XHR.open( "GET", `/settings?action=email&emailaddr=${email}`, true );  
  XHR.onloadend = function(){
    alert( XHR.responseText );
  }
  XHR.send( null );
}
)rawliteral";

  sprintf(the_page, msg1, remoteTrigger);

  // send first chunk
  httpd_resp_send_chunk(req, the_page, strlen(the_page));

  strcpy(the_page, ""); // clear page for next chunk

  const char msg2[] PROGMEM = R"rawliteral(

var curpir = %i;

function do_piremail(action) {
  if ( action == 0 ) { // display current value
    if ( curpir < 10 ) { // pir email is off
      document.getElementById('piremail').innerHTML = '<a href="javascript:do_piremail(1);">Turn On PIR Email</a>';
    } else { // pir email is on
      document.getElementById('piremail').innerHTML = '<a href="javascript:do_piremail(1);">Turn Off PIR Email</a>';    
    }
  } else { // toggle value
    if ( curpir < 10 ) { // turn on motion detection
      curpir = curpir + 10;
      document.getElementById('piremail').innerHTML = '<a href="javascript:do_piremail(1);">Turn Off PIR Email</a>';
    } else { // turn off motion detection
      curpir = curpir - 10;
      document.getElementById('piremail').innerHTML = '<a href="javascript:do_piremail(1);">Turn On PIR Email</a>';    
    }
    var XHR = new XMLHttpRequest();
    XHR.open( "GET", `/settings?action=piremail&piraction=${curpir}`, true );   
    XHR.onloadend = function(){
      alert( XHR.responseText );
    }
    XHR.send( null );
  }
}
        
function do_piraction(action) {
  var a = curpir; // a is action
  if ( a > 9 ) 
    a = a - 10;
  var of = 0; // of is on off value
  if ( curpir > 9 ) 
    of = 10;
  if ( action == 0 ) { // display current value
    document.getElementById("piraction").selectedIndex = a - 1;
  } else { // process a possible change
    var e = document.getElementById("piraction");
    var na = parseInt(e.options[e.selectedIndex].value);
    if ( na != a ) { // drop down selection has changed
      curpir = of + na;
      var XHR = new XMLHttpRequest();
      XHR.open( "GET", `/settings?action=piremail&piraction=${curpir}`, true );   
      XHR.onloadend = function(){
        alert( XHR.responseText );
      }
      XHR.send( null );
    }
  }
}

var curwakeuptime = %i;

function do_wakeupaction(action) {
  var a = 0; // a is action value
  var ctime = curwakeuptime %% 10000;
  if ( curwakeuptime > 9999 ) 
    a = 1;
  if ( action == 0 ) { // display current value
    document.getElementById("wakeupaction").selectedIndex = a;
  } else { // process a possible change
    var e = document.getElementById("wakeupaction");
    var na = parseInt(e.options[e.selectedIndex].value) - 1;
    if ( na != a ) { // drop down selection has changed
      curwakeuptime = (na * 10000) + ctime;
      var XHR = new XMLHttpRequest();
      XHR.open( "GET", `/settings?action=wakeup&wakeupaction=${curwakeuptime}`, true );   
      XHR.onloadend = function(){
        alert( XHR.responseText );
      }
      XHR.send( null );
    }
  }
}

var curth = %i;
var curtm = %i;

function do_time(action) {
  if ( action == 0 ) {
    document.getElementById("th").value = curth;
    document.getElementById("tm").value = curtm;
  } else {
    // send changes when changes are done
    var th = parseInt(document.getElementById("th").value);
    var tm = parseInt(document.getElementById("tm").value);
    clearTimeout(timeout);
    if ( th > -1 && th < 24 && tm > -1 && tm < 60 ) {
      var newwakeuptime = (th * 100) + tm;
      if (curwakeuptime > 9999)
        newwakeuptime = newwakeuptime + 10000;
      curth = th;
      curtm = tm;
      curwakeuptime = newwakeuptime;
      document.getElementById("th").value = th;
      document.getElementById("tm").value = tm;
      timeout = setTimeout(function () {
        var XHR = new XMLHttpRequest();
        XHR.open( "GET", `/settings?action=wakeuptime&th=${th}&tm=${tm}`, true );            
        XHR.onloadend = function(){
          alert( XHR.responseText );
        }
        XHR.send( null );
      }, 4000);
    } else {
      document.getElementById("th").value = curth;  
      document.getElementById("tm").value = curtm;  
    }
  }
}

</script>
</head>
)rawliteral";

  int TH = ((wakeupTime % 10000) / 100);
  int TM = (wakeupTime % 100);

  sprintf(the_page, msg2, motionEmailAction, wakeupTime, TH, TM);

  // send second chunk
  httpd_resp_send_chunk(req, the_page, strlen(the_page));

  strcpy(the_page, ""); // clear page for next chunk
 
  const char msg3[] PROGMEM = R"rawliteral(
        
<body>
<center>
<h1>ESP32-CAM Remote PIR</h1>
 <h2>Settings</h2>
 <table id="actions">
   <tr><td>All video recordings default to VGA (640x480), one frame every %ims for 10 seconds.<br>
     Upon PIR wakeup a remote trigger can be sent to the IP address below.<br>
     A web server will wakeup once per day for 10 minutes to allow remote configuration.<br>
   </td></tr>
 </table>   
 <table id="actions">
   <tr><td>&nbsp</td></tr>
   <tr><td><span id="remote"><a href="javascript:do_remote(1);">Turn On Remote Trigger</a></span>
     &nbsp;Send remote trigger to IP address:
     <input type="text" id="remoteip" value="%s" maxlength="15" size="15" onchange="do_remoteip();"><br>
     Remote trigger action:
     <select id="remoteaction" onchange="do_remoteaction(1);">
       <option value="1">Take a picture</option>
       <option value="2">Take a picture and email it</option>
       <option value="3">Take a video</option>
       <option value="4">Take a video and email video name</option>
     </select>&nbsp;
     <a href="javascript:do_remotetest();">Test Remote Trigger</a>&nbsp;Wait for it to complete.<br>
     </td></tr>
   <tr><td>&nbsp</td></tr>
   <tr><td>Set email address: 
     <input type="text" id="email" value="%s" maxlength="40" size="40">&nbsp;
     <a href="javascript:do_email();">Send Test Email</a><br>
     Wait 15 seconds for the test to complete. 
     The new email address will be saved if the test succeeds.</td></tr>
   <tr><td>&nbsp</td></tr>
   <tr><td><span id="piremail"><a href="javascript:do_piremail(1);">Turn On PIR Email</a></span>
     &nbsp;PIR email action:
     <select id="piraction" onchange="do_piraction(1);">
       <option value="1">Email one picture</option>
       <option value="2">Email two pictures</option>
       <option value="3">Email three pictures</option>
       <option value="4">Email four pictures</option>
       <option value="5">Email five pictures</option>
       <option value="6">Email six pictures</option>
       <option value="7">Email a 10 second video</option>
     </select><br>
     Pictures are one second apart.
   </td></tr>
   <tr><td>&nbsp</td></tr>
   <tr><td>Web server wakeup action: 
     <select id="wakeupaction" onchange="do_wakeupaction(1);">
       <option value="1">Run web server only</option>
       <option value="2">Run web server and email a picture</option>
     </select>
     <br>Web server wakeup time (0-23H : 0-59M):
     <input type="number" id="th" value="0" min="0" max="23" style="width: 3em" onchange="do_time(1);"/>
     H :
     <input type="number" id="tm" value="0" min="0" max="59" style="width: 3em" onchange="do_time(1);"/>
     M
     </td></tr>
   <tr><td>&nbsp</td></tr>
   <tr><td><a href="javascript:do_reset();">ESP32-CAM Reboot</a></td></tr>
 </table>
</body></html>)rawliteral";

  sprintf(the_page, msg3, capture_interval, remoteTriggerIP, email);

  // send last chunks
  httpd_resp_send_chunk(req, the_page, strlen(the_page));

  httpd_resp_send_chunk(req, NULL, 0);

}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 
static esp_err_t settings_handler(httpd_req_t *req) {

  timeout_10min = millis();

  char buf[500];
  size_t buf_len;
  char param[60];
  char action[20];
  int newremoteTrigger;
  char newremoteTriggerIP[16];
  char newemail[60];
  int newmotionEmailAction;
  int newwakeupTime;
  int newwakeupH;
  int newwakeupM;

  strcpy(action, "show");

  // query parameters - get action
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "action", param, sizeof(param)) == ESP_OK) {
        strcpy(action, param);
      }
      if (httpd_query_key_value(buf, "raction", param, sizeof(param)) == ESP_OK) {
        newremoteTrigger = atoi( param );
      }
      if (httpd_query_key_value(buf, "remoteipaddr", param, sizeof(param)) == ESP_OK) {
        strcpy(newremoteTriggerIP,param);
      }
      if (httpd_query_key_value(buf, "emailaddr", param, sizeof(param)) == ESP_OK) {
        strcpy(newemail,param);
      }
      if (httpd_query_key_value(buf, "piraction", param, sizeof(param)) == ESP_OK) {
        newmotionEmailAction = atoi( param );
      }
      if (httpd_query_key_value(buf, "wakeupaction", param, sizeof(param)) == ESP_OK) {
        newwakeupTime = atoi( param );
      }
      if (httpd_query_key_value(buf, "th", param, sizeof(param)) == ESP_OK) {
        newwakeupH = atoi( param );
      }
      if (httpd_query_key_value(buf, "tm", param, sizeof(param)) == ESP_OK) {
        newwakeupM = atoi( param );
      }
    }
  }

  if ( !strcmp( action, "remote" ) ) {
    // set remote trigger action stuff 
    EEPROM.put(EEPROM_REMOTE_ADDR, newremoteTrigger);
    EEPROM.commit();
    if ( abs( newremoteTrigger - remoteTrigger ) > 9 ) { // remote trigger turned on or off
      remoteTrigger = newremoteTrigger;
      if ( newremoteTrigger < 10 ) {
        sprintf(the_page, "Remote trigger turned off");
      } else {
        sprintf(the_page, "Remote trigger turned on");
      }
    } else { // Remote trigger action changed
      remoteTrigger = newremoteTrigger;
      if ( newremoteTrigger > 9 )
        newremoteTrigger = newremoteTrigger - 10;
      if ( newremoteTrigger == 0 )
        sprintf(the_page, "Remote trigger action set to Take a picture");
      if ( newremoteTrigger == 1 )
        sprintf(the_page, "Remote trigger action set to Take a picture and email it");
      if ( newremoteTrigger == 2 )
        sprintf(the_page, "Remote trigger action set to Take a video");
      if ( newremoteTrigger == 3 )
        sprintf(the_page, "Remote trigger action set to Take a video and email name of video");
    }
    Serial.println( the_page );
    httpd_resp_send(req, the_page, strlen(the_page));
    
  } else if ( !strcmp( action, "remoteip" ) ) {
    // set remote trigger ip 
    EEPROM.put(EEPROM_REMOTEIP_ADDR, newremoteTriggerIP);
    EEPROM.commit();
    strcpy(remoteTriggerIP, newremoteTriggerIP);
    sprintf(the_page, "Remote trigger IP set to %s", remoteTriggerIP);
    Serial.println( the_page );
    httpd_resp_send(req, the_page, strlen(the_page));
    
  } else if ( !strcmp( action, "remotetest" ) ) {
    // send remote trigger to ip 
    char url[100];
    sprintf(url,"http://%s/remotetrigger?action=%i", remoteTriggerIP, 10 + (remoteTrigger % 10));
    String response = send_http_url( url );  
    sprintf(the_page, "Remote trigger sent to %s\n%s", remoteTriggerIP, response.c_str());
    Serial.println( the_page );
    httpd_resp_send(req, the_page, strlen(the_page));

  } else if ( !strcmp( action, "email" ) ) {
    static char sub[30];
    strcpy( sub, "ESP32-CAM Test Email" );
    static char msg[50];
    strcpy( msg, "This is a test email\n" );
    delay(3);

    // create SendEmail object 
    SendEmail e(emailhost, emailport, emailsendaddr, emailsendpwd, 5000, true); 
   
    char oldemail[40];
    strcpy(oldemail, newemail);
    strcpy(email, newemail);
    //Serial.println("SendEmail object created, now trying to send...");

    if ( email_pictures( sub, msg, 1 ) ) {
      EEPROM.put(EEPROM_EMAIL_ADDR, email);
      EEPROM.commit();
      sprintf(the_page, "New email address is %s", email);    
    } else {
      strcpy(email,oldemail);
      sprintf(the_page, "New email address test failed");    
    }

    e.close(); // close email client
    Serial.println( the_page );
    httpd_resp_send(req, the_page, strlen(the_page));

  } else if ( !strcmp( action, "piremail" ) ) {
    EEPROM.put(EEPROM_MOTIONEMAILACTION_ADDR, newmotionEmailAction);
    EEPROM.commit();
    if ( abs( newmotionEmailAction - motionEmailAction ) > 9 ) { // pir email turned on or off
      motionEmailAction = newmotionEmailAction;
      if ( newmotionEmailAction < 10 ) {
        sprintf(the_page, "PIR email turned off");
      } else {
        sprintf(the_page, "PIR email turned on");
      }
    } else { // PIR email action changed
      motionEmailAction = newmotionEmailAction;
      if ( newmotionEmailAction > 9 )
        newmotionEmailAction = newmotionEmailAction - 10;
      if ( newmotionEmailAction == 1 )
        sprintf(the_page, "PIR action set to Email one picture");
      if ( newmotionEmailAction == 2 )
        sprintf(the_page, "PIR action set to Email two pictures");
      if ( newmotionEmailAction == 3 )
        sprintf(the_page, "PIR action set to Email three pictures");
      if ( newmotionEmailAction == 4 )
        sprintf(the_page, "PIR action set to Email four pictures");
      if ( newmotionEmailAction == 5 )
        sprintf(the_page, "PIR action set to Email five pictures");
      if ( newmotionEmailAction == 6 )
        sprintf(the_page, "PIR action set to Email six pictures");
      if ( newmotionEmailAction == 7 )
        sprintf(the_page, "PIR action set to Email one video");
    }
    Serial.println( the_page );
    httpd_resp_send(req, the_page, strlen(the_page));

  } else if ( !strcmp( action, "wakeup" ) ) {
    EEPROM.put(EEPROM_WAKEUPTIME_ADDR, newwakeupTime);
    EEPROM.commit();
    wakeupTime = newwakeupTime;
    if ( newwakeupTime < 10000 ) {
      sprintf(the_page, "Web Server action set to Run web server only");
    } else {
      sprintf(the_page, "Web Server action set to Run web server and email a picture");
    }
    Serial.println( the_page );
    httpd_resp_send(req, the_page, strlen(the_page));

  } else if ( !strcmp( action, "wakeuptime" ) ) {
    if ( newwakeupH > -1 && newwakeupH < 24 & newwakeupM > -1 && newwakeupM < 60 ) {
      // set new wakeup time      
      wakeupTime = ((wakeupTime / 10000) * 10000) + (newwakeupH * 100) + newwakeupM;
      EEPROM.put(EEPROM_WAKEUPTIME_ADDR, wakeupTime);
      EEPROM.commit();
      sprintf(the_page, "Wakeup time set to %i H : %i M", newwakeupH, newwakeupM);
      Serial.println( the_page );
      httpd_resp_send(req, the_page, strlen(the_page));
    }
  } else if ( !strcmp( action, "reset" ) ) {
    // call reset handler
    reset_handler(req);
  } else {
    // display settings page
    do_settings(req);
  }
  
  return ESP_OK;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 

static esp_err_t updatefirmware_get_handler(httpd_req_t *req) {

  Serial.println(F("In updatefirmware_get_handler"));
  timeout_10min = millis();

  const char msg[] PROGMEM = R"rawliteral(<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Firmware Updater</title>
<script>

function clear_status() {
  document.getElementById('status').innerHTML = " &nbsp; ";
}

function uploadfile(file) {
  if ( !document.getElementById("updatefile").value ) {
    alert( "Choose a valid firmware file" );
  } else {
    let xhr = new XMLHttpRequest();
    document.getElementById('updatebutton').disabled = true;
    document.getElementById('status').innerText = "Progress 0%%";
    let eraseEEPROMvalue = document.getElementById('EraseEEPROM').checked;
    let upwd = document.getElementById('upwd').value;
    // track upload progress
    xhr.upload.onprogress = function(event) {
      if (event.lengthComputable) {
        var per = event.loaded / event.total;
        document.getElementById('status').innerText = "Progress " + Math.round(per*100) + "%%";
      }
    };
    // track completion: both successful or not
    xhr.onloadend = function() {
      if (xhr.status == 200) {
        document.getElementById('status').innerText = xhr.response;
      } else {
        document.getElementById('status').innerText = "Firmware update failed";
      }
      document.getElementById('updatebutton').disabled = false;
      document.getElementById('upwd').value = "";
    };
    xhr.open("POST", "/updatefirmware");
    xhr.setRequestHeader('EraseEEPROM', eraseEEPROMvalue);
    xhr.setRequestHeader('UPwd', upwd);
    xhr.send(file);
  }
}

</script>
</head>
<body><center>
<h1>ESP32 Firmware Updater</h1>

Select an ESP32 firmware file (.bin) to update the ESP32 firmware<br><br>

<table>
<tr><td align="center"><input type="file" id="updatefile" accept=".bin" onclick="clear_status();"><br><br></td></tr>
<tr><td align="center"><input type="checkbox" id="EraseEEPROM" onclick="clear_status();"> Erase EEPROM<br><br></td></tr>
<tr><td align="center">Update Password <input type="password" id="upwd" maxlength="20"><br><br></td></tr>
<tr><td align="center"><input type="button" id="updatebutton" onclick="uploadfile(updatefile.files[0]);" value="Update"><br><br></td></tr>
<tr><td align="center"><div id="status"> &nbsp; </div><br><br></td></tr>
<tr><td align="center">%s Version %s</td></tr>
</table>
</center></body>
</html>)rawliteral";

  //strcpy(the_page, msg);
  sprintf(the_page, msg, appName, appVersion);

  httpd_resp_send(req, the_page, strlen(the_page));

  return ESP_OK;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 
#define UPLOAD_CACHE_SIZE 1600

static esp_err_t updatefirmware_post_handler(httpd_req_t *req) {

  Serial.println(F("In updatefirmware_post_handler"));
  timeout_10min = millis();

  char contentBuffer[UPLOAD_CACHE_SIZE];
  size_t recv_size = sizeof(contentBuffer);
  size_t contentLen = req->content_len;

  char eraseEeprom[10];
  httpd_req_get_hdr_value_str(req, "EraseEEPROM", eraseEeprom, sizeof(eraseEeprom));
  Serial.println((String) "EraseEEPROM " + eraseEeprom );
  char upwd[20];
  httpd_req_get_hdr_value_str(req, "UPwd", upwd, sizeof(upwd));
  Serial.println((String) "Update password " + upwd );

  Serial.println((String) "Content length is " + contentLen);

  if ( !strcmp( firmwareUpdatePassword, upwd ) ) {
    // update password is good, do the firmware update

    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start flash with max available size
      Update.printError(Serial);
      strcpy(the_page, "Firmware update failed - Update failed begin step");
      Serial.println( the_page );
      httpd_resp_send(req, the_page, strlen(the_page));
      return ESP_OK;
    }
      
    size_t bytes_recvd = 0;
    while (bytes_recvd < contentLen) {
      //if ((contentLen - bytes_recvd) < recv_size) recv_size = contentLen - bytes_recvd;
      int ret = httpd_req_recv(req, contentBuffer, recv_size);
      if (ret <= ESP_OK) {  /* ESP_OK return value indicates connection closed */
        /* Check if timeout occurred */
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
          httpd_resp_send_408(req);
        }
        return ESP_FAIL;
      }
      if (Update.write((uint8_t *) contentBuffer, ret) != ret) {
        Update.printError(Serial);
        strcpy(the_page, "Firmware update failed - Update failed write step");
        Serial.println( the_page );
        httpd_resp_send(req, the_page, strlen(the_page));
        return ESP_OK;
      }
      bytes_recvd += ret;
    }

    if (!Update.end(true)) { //true to set the size to the current progress
      Update.printError(Serial);
      strcpy(the_page, "Firmware update failed - Update failed end step");
      Serial.println( the_page );
      httpd_resp_send(req, the_page, strlen(the_page));
      return ESP_OK;
    }

    if ( !strcmp( "true", eraseEeprom ) ) { // erase EEPROM
      EEPROM.end();
      EEPROM.begin(512);
      for (int i = 0 ; i < 512 ; i++) {
        EEPROM.write(i, 0);  // set all EEPROM memory to 0
      }
      EEPROM.end();
      strcpy(the_page, "Firmware update and EEPROM erase successful - Rebooting");
    } else {
      strcpy(the_page, "Firmware update successful - Rebooting");
    }
    Serial.println( the_page );
    httpd_resp_send(req, the_page, strlen(the_page));

    delay(5000);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP1S * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  } else {
    strcpy(the_page, "Firmware update failed - Invalid password");
  }

  Serial.println( the_page );
  httpd_resp_send(req, the_page, strlen(the_page));
  return ESP_OK;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 

void startWebServer() {

  // start a web server for OTA updates
  // this same web server can be used for all other normal web server stuff
  // just add the appropriate uri handlers

  // allocate 8000 from PRAM for holding response page before its sent to browser
  the_page = (char*) heap_caps_calloc(8000, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = SERVER_PORT;
  config.max_uri_handlers = 10;
  config.max_resp_headers = 10;
  //config.stack_size = 12288;
  config.stack_size = 16384;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t settings_uri = {
    .uri       = "/settings",
    .method    = HTTP_GET,
    .handler   = settings_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t updatefirmware_get_uri = {
    .uri       = "/updatefirmware",
    .method    = HTTP_GET,
    .handler   = updatefirmware_get_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t updatefirmware_post_uri = {
    .uri       = "/updatefirmware",
    .method    = HTTP_POST,
    .handler   = updatefirmware_post_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&webserver_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(webserver_httpd, &index_uri);
    httpd_register_uri_handler(webserver_httpd, &stream_uri);
    httpd_register_uri_handler(webserver_httpd, &settings_uri);
    httpd_register_uri_handler(webserver_httpd, &updatefirmware_get_uri);
    httpd_register_uri_handler(webserver_httpd, &updatefirmware_post_uri);
  }
}

/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 

// MicroSD
#include "SD_MMC.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"

static esp_err_t init_sdcard()
{
  esp_err_t ret = ESP_FAIL;
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    //.max_files = 10,
    .max_files = 6,
  };
  sdmmc_card_t *card;

  Serial.println("Mounting SD card...");
  ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

  if (ret == ESP_OK) {
    Serial.println("SD card mount successfully!");
  }  else  {
    Serial.printf("Failed to mount SD card VFAT filesystem. Error: %s", esp_err_to_name(ret));
    major_fail();
  }

  Serial.print("SD_MMC Begin: "); Serial.println(SD_MMC.begin());
}
*/


TaskHandle_t RemoteTriggerTask;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 
void codeForRemoteTriggerTask( void * parameter ) {
  PIRActionActive = 1;
  if ( remoteTrigger > 9 && PIRWakeup == 1 ) {
    // send remote trigger to ip 
    char url[100];
    sprintf(url,"http://%s/remotetrigger?action=%i", remoteTriggerIP, 10 + (remoteTrigger % 10));
    String response = send_http_url( url );  
    Serial.print( "Remote trigger sent to " ); Serial.println( remoteTriggerIP );
    Serial.println( response.c_str() );
  }
  PIRActionActive = 0;
  delay(100);
  vTaskDelete(RemoteTriggerTask);
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 
void PIR_wakeup() {
  Serial.println(F("Wakeup caused by PIR"));

  unsigned long timeout_25sec = millis();

  Serial.print(" PIR_wakeup before start Remote trigger  Free Heap: "); Serial.println(ESP.getFreeHeap());

  PIRActionActive = 0;
  // do remote trigger action
  xTaskCreatePinnedToCore(
    codeForRemoteTriggerTask,
    "RemoteTriggerTask",
    12000,
    NULL,
    1,
    &RemoteTriggerTask,
    0);

  Serial.print(" PIR_wakeup before PIR action  Free Heap: "); Serial.println(ESP.getFreeHeap());

  // do PIR action
  if ( motionEmailAction > 9 ) {

    if ( motionEmailAction < 17 ) {  // do pictures email
      static char sub[30];
      strcpy( sub, "Camera Event Detected" );
      static char msg[120];
      strcpy( msg, "PIR trigger, please see attached pictures\n" );
      strcat( msg, batteryMsg );
      int pictureCount = (motionEmailAction % 10);
      email_pictures( sub, msg, pictureCount );

    } else {  // do 10 second video email
      static char sub[30];
      strcpy( sub, "Camera Event Detected" );
      static char msg[120];
      strcpy( msg, "PIR trigger, please see attached video\n" );
      strcat( msg, batteryMsg );
      email_video( sub, msg );
    }
  }

  Serial.print(" PIR_wakeup after PIR action  Free Heap: "); Serial.println(ESP.getFreeHeap());

  while ( PIRActionActive == 1 ) delay(100);  // wait for PIR remote trigger to finish

  Serial.print(" PIR_wakeup waiting to sleep  Free Heap: "); Serial.println(ESP.getFreeHeap());

  while (millis() - timeout_25sec < (25 * 1000) ) {       // 25 second timeout
    delay(100);
  }
  
  do_deep_sleep();
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 

void config_wakeup() {
  Serial.println(F("Wakeup caused by timer, power on, or reset - starting config web server"));  

  Serial.print(" Before startWebServer  Free Heap: "); Serial.println(ESP.getFreeHeap());

  startWebServer();

  Serial.print(" After startWebServer  Free Heap: "); Serial.println(ESP.getFreeHeap());

  if ( (wakeupTime / 10000) == 1 ) {  // send a wakeup email
    static char sub[30];
    strcpy( sub, "Camera Wakeup Event" );
    static char msg[120];
    strcpy( msg, "Remote PIR will be available for 10 minutes\n" );
    strcat( msg, batteryMsg );
    email_pictures( sub, msg, 1 );
  }

  // start a timer to put into deep sleep after 10 minutes
  timeout_10min = millis();
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// setup() runs on cpu 1

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);

  pinMode(33, OUTPUT);    // little red led on back of chip
  digitalWrite(33, LOW);  // turn on the red LED on the back of chip

  emailvideostatus = 0;

  if ( PIRWakeup == 0 ) {  // fresh wakeup, get wakeup reason
    // get wakeup reason
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();
    switch(wakeup_reason)
    {
      case ESP_SLEEP_WAKEUP_EXT0 : PIRWakeup = 1; break;
      default : PIRWakeup = 0; break;
    }
  }

  // Configure GPIO12 for checking battery voltage and get current reading
  Configure_GPIO12();
  batteryVoltage = Get_GPIO12_Voltage();
  sprintf( batteryMsg, "The battery voltage is %dmV", batteryVoltage );
  Serial.println( batteryMsg );
  
  if (init_wifi()) { // Connected to WiFi
    Serial.println(F("Internet connected"));

    init_time();
    time(&now);

    // set timezone
    setenv("TZ", TZ_INFO, 1); 
    tzset();
    time(&now);
    localtime_r(&now, &timeinfo);

    Serial.print("After timezone : "); Serial.println(ctime(&now));
  }

/*  
  // SD card init
  esp_err_t card_err = init_sdcard();
  if (card_err != ESP_OK) {
    Serial.printf("SD Card init failed with error 0x%x", card_err);
    major_fail();
    return;
  }
*/

  // load saved values from EEPROM
  EEPROM.begin(EEPROM_SIZE_ADDR);
  int eeprom_init = 0; // 0 is ready, 1 is not ready
  for(int b = EEPROM_READY_ADDR; b < EEPROM_READY_ADDR + sizeof(eeprom_ready); b++) {
    // EEPROM = Init Code?
    if(EEPROM.read(b) != eeprom_ready[b]) {
      // EEPROM not ready, set Ready
      EEPROM.write(b, eeprom_ready[b]);
      // set eeprom_init flag to 1
      eeprom_init = 1;
    }
  }
  if( eeprom_init == 0 ) { // EEPROM is ready, load values from it
    Serial.println(F("Loading initial values from EEPROM"));
    EEPROM.get(EEPROM_REMOTE_ADDR, remoteTrigger);
    Serial.print( F("Remote trigger set to ") ); Serial.println( remoteTrigger );
    EEPROM.get(EEPROM_REMOTEIP_ADDR, remoteTriggerIP);
    Serial.print( F("Remote IP address set to ") ); Serial.println( remoteTriggerIP );
    EEPROM.get(EEPROM_EMAIL_ADDR, email);
    Serial.print( F("Email address set to ") ); Serial.println( email );
    EEPROM.get(EEPROM_MOTIONEMAILACTION_ADDR, motionEmailAction);
    Serial.print( ("Motion email action set to ") ); Serial.println( motionEmailAction );
    EEPROM.get(EEPROM_WAKEUPTIME_ADDR, wakeupTime);
    Serial.print( F("Wakeup Time set to ") ); Serial.println( wakeupTime );
  } else { // EEPROM not ready, save default values
    Serial.println( F("First run after flash, saving default values in EEPROM") );
    EEPROM.put(EEPROM_REMOTE_ADDR, remoteTrigger);
    EEPROM.put(EEPROM_REMOTEIP_ADDR, remoteTriggerIP);
    EEPROM.put(EEPROM_EMAIL_ADDR, email);
    EEPROM.put(EEPROM_MOTIONEMAILACTION_ADDR, motionEmailAction);
    EEPROM.put(EEPROM_WAKEUPTIME_ADDR, wakeupTime);
    EEPROM.commit();
  }


  baton = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    codeForCameraTask,
    "CameraTask",
    10000,
    NULL,
    1,
    &CameraTask,
    0);

  xTaskCreatePinnedToCore(
    codeForAviWriterTask,
    "AviWriterTask",
    10000,
    NULL,
    2,
    &AviWriterTask,
    1);

  // take a action based on wakeup reason
  if ( PIRWakeup > 0 ) {
    PIR_wakeup();
  } else {
    config_wakeup();
  }

  pinMode(GPIO_NUM_13, INPUT_PULLUP);   // enable pin as an INPUT pin, could also set as OUTPUT, INPUT_PULLUP
              // note: in this build high on this pin indicates no motion, low indicates motion

  digitalWrite(33, HIGH);  // turn off the red LED on the back of chip

  Serial.print(" End of setup  Free Heap: "); Serial.println(ESP.getFreeHeap());
  Serial.print( F("ESP32 Ready! Use 'http://") );
  Serial.print(WiFi.localIP());
  Serial.println( F("' to connect") );

}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// 

void loop() {
  
  if ( millis() - timeout_10min > (10 * 60 * 1000) ) {       // 10 minutes
    do_deep_sleep();
  }

  if ( emailvideostatus == 1 ) {  // takes too long to keep in uri handler so doing it here
    static char sub[30];
    strcpy( sub, "Camera Event" );
    static char msg[120];
    strcpy( msg, "Manual trigger, please see attached video\n" );
    strcat( msg, batteryMsg );

    //Serial.print(F("in EmailVideoTask, Heap is: ")); Serial.println(ESP.getFreeHeap());
  
    email_video( sub, msg );
    // after email_video emailvideostatus should be set to 2 or 3
  }
  
  delay(1000);
}


/*****************************************************************************
 * 
 *  From here to the end is the contents of sendemail.cpp
 *  This version can send text or binary objects (jpg, avi) from memory as attachments
 *  and can send files from the SD card mounted on /sdcard.
 *  
 *  *** Uncomment the #include if the below is moved back into sendemail.cpp ***
 *
 */

//#include "sendemail.h"

SendEmail::SendEmail(const String& host, const int port, const String& user, const String& passwd, const int timeout, const bool ssl) :
    host(host), port(port), user(user), passwd(passwd), timeout(timeout), ssl(ssl), client((ssl) ? new WiFiClientSecure() : new WiFiClient())
{
  attachbuffercount = 0;
#ifdef USING_SD_CARD
  attachfilecount = 0;
#endif
}

String SendEmail::readClient()
{
  String r = client->readStringUntil('\n');
  r.trim();
  while (client->available()) r += client->readString();
  return r;
}

void SendEmail::attachbuffer(char * name, char * bufptr, size_t bufsize)
{
  attachbufferitems[attachbuffercount].buffername = (char *) malloc( strlen(name)+1 );
  strcpy( attachbufferitems[attachbuffercount].buffername, name ); 
  attachbufferitems[attachbuffercount].buffer = bufptr;
  attachbufferitems[attachbuffercount].buffersize = bufsize;
  
  attachbuffercount++;
}

#ifdef USING_SD_CARD
void SendEmail::attachfile(char * name)
{
  attachfileitems[attachfilecount].filename = (char *) malloc( strlen(name)+8 );
  strcpy( attachfileitems[attachfilecount].filename, "/sdcard" );
  strcat( attachfileitems[attachfilecount].filename, name );
  
  attachfilecount++;
}
#endif


bool SendEmail::send(const String& from, const String& to, const String& subject, const String& msg)
{
  if (!host.length())
  {
    return false;
  }
  String buffer2((char *)0);
  buffer2.reserve(800);  // really should only use 780 of it
  client->stop();
  client->setTimeout(timeout);
  // smtp connect
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("Connecting: "));
  Serial.print(host);
  Serial.print(":");
  Serial.println(port);
#endif
  if (!client->connect(host.c_str(), port))
  {
    return false;
  }
  String buffer = readClient();
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("SERVER->CLIENT: "));
  Serial.println(buffer);
#endif
  // note: F(..) as used below puts the string in flash instead of RAM
  if (!buffer.startsWith(F("220")))
  {
    return false;
  }
  buffer = F("HELO ");
  buffer += client->localIP();
  client->println(buffer);
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("CLIENT->SERVER: "));
  Serial.println(buffer);
#endif
  buffer = readClient();
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("SERVER->CLIENT: "));
  Serial.println(buffer);
#endif
  if (!buffer.startsWith(F("250")))
  {
    return false;
  }
  if (user.length()>0  && passwd.length()>0 )
  {
    buffer = F("AUTH LOGIN");
    client->println(buffer);
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("CLIENT->SERVER: "));
  Serial.println(buffer);
#endif
    buffer = readClient();
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("SERVER->CLIENT: "));
  Serial.println(buffer);
#endif
    if (!buffer.startsWith(F("334")))
    {
      return false;
    }
    base64 b;
    buffer = user;
    buffer = b.encode(buffer);
    client->println(buffer);
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("CLIENT->SERVER: "));
  Serial.println(buffer);
#endif
    buffer = readClient();
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("SERVER->CLIENT: "));
  Serial.println(buffer);
#endif
    if (!buffer.startsWith(F("334")))
    {
      return false;
    }
    buffer = this->passwd;
    buffer = b.encode(buffer);
    client->println(buffer);
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("CLIENT->SERVER: "));
  Serial.println(buffer);
#endif
    buffer = readClient();
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("SERVER->CLIENT: "));
  Serial.println(buffer);
#endif
    if (!buffer.startsWith(F("235")))
    {
      return false;
    }
  }
  // smtp send mail
  buffer = F("MAIL FROM: ");
  buffer += from;
  client->println(buffer);
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("CLIENT->SERVER: "));
  Serial.println(buffer);
#endif
  buffer = readClient();
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("SERVER->CLIENT: "));
  Serial.println(buffer);
#endif
  if (!buffer.startsWith(F("250")))
  {
    return false;
  }
  buffer = F("RCPT TO: ");
  buffer += to;
  client->println(buffer);
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("CLIENT->SERVER: "));
  Serial.println(buffer);
#endif
  buffer = readClient();
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("SERVER->CLIENT: "));
  Serial.println(buffer);
#endif
  if (!buffer.startsWith(F("250")))
  {
    return false;
  }
  buffer = F("DATA");
  client->println(buffer);
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("CLIENT->SERVER: "));
  Serial.println(buffer);
#endif
  buffer = readClient();
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("SERVER->CLIENT: "));
  Serial.println(buffer);
#endif
  if (!buffer.startsWith(F("354")))
  {
    return false;
  }
  buffer = F("From: ");
  buffer += from;
  client->println(buffer);
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("CLIENT->SERVER: "));
  Serial.println(buffer);
#endif
  buffer = F("To: ");
  buffer += to;
  client->println(buffer);
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("CLIENT->SERVER: "));
  Serial.println(buffer);
#endif
  buffer = F("Subject: ");
  buffer += subject;
  client->println(buffer);
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("CLIENT->SERVER: "));
  Serial.println(buffer);
#endif
  // setup to send message body
  buffer = F("MIME-Version: 1.0\r\n");
  buffer += F("Content-Type: multipart/mixed; boundary=\"{BOUNDARY}\"\r\n\r\n");
  buffer += F("--{BOUNDARY}\r\n");
  buffer += F("Content-Type: text/plain\r\n");
  client->println(buffer);
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("CLIENT->SERVER: "));
  Serial.println(buffer);
#endif
  buffer = msg;
  client->println(buffer);
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("CLIENT->SERVER: "));
  Serial.println(buffer);
#endif
  // process buffer attachments
  for ( int i = 0; i < attachbuffercount; i++ ) {
    char * pos = attachbufferitems[i].buffer;
    size_t alen = attachbufferitems[i].buffersize;
    base64 b;
    
    buffer = F("\r\n--{BOUNDARY}\r\n");
    buffer += F("Content-Type: application/octet-stream\r\n");
    buffer += F("Content-Transfer-Encoding: base64\r\n");
    buffer += F("Content-Disposition: attachment;filename=\"");
    buffer += attachbufferitems[i].buffername;
    buffer += F("\"\r\n");
    client->println(buffer);
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("CLIENT->SERVER: "));
  Serial.println(buffer);
  size_t bytessent = 0;
#endif
    // read data from buffer, base64 encode and send it
    // 3 binary bytes (57) becomes 4 base64 bytes (76)
    // plus CRLF is ideal for one line of MIME data
    // 570 bytes will be read at a time and sent as ten lines of base64 data
    size_t flen = 570;  
    uint8_t * fdata = (uint8_t*) malloc( flen );
    if ( alen < flen ) flen = alen;
    // read data from buffer
    memcpy( fdata, pos, flen ); 
    delay(10);
    buffer2 = "";
    size_t bytecount = 0;
    while ( flen > 0 ) {
      while ( flen > 56 ) {
        // convert to base64 in 57 byte chunks
        buffer2 += b.encode( fdata+bytecount, 57 );
        // tack on CRLF
        buffer2 += "\r\n";
        bytecount += 57;
        flen -= 57;
      }
      if ( flen > 0 ) {
        // convert last set of bytes to base64
        buffer2 += b.encode( fdata+bytecount, flen );
        // tack on CRLF
        buffer2 += "\r\n";
      }
      // send the lines in buffer
      client->println( buffer2 );
#ifdef DEBUG_EMAIL_PORT
  bytessent += bytecount + flen;
  Serial.print(F(" bytes sent so far: ")); Serial.println(bytessent);
#endif
      buffer2 = "";
      delay(10);
      // calculate bytes left to send
      alen = alen - bytecount - flen;
      pos = pos + bytecount + flen;
      flen = 570;
      if ( alen < flen ) flen = alen;
      // read data from buffer
      if ( flen > 0 ) memcpy( fdata, pos, flen ); 
      bytecount = 0;
    }
    free( fdata );
    fdata = NULL;
  }

#ifdef USING_SD_CARD
  // process file attachments
  for ( int i = 0; i < attachfilecount; i++ ) {
    FILE *atfile = NULL;
    char aname[110];
    char * pos = NULL;  // points to actual file name
    size_t flen;
    base64 b;
    if ( atfile = fopen(attachfileitems[i].filename, "r") ) {
      // get filename from attachment name
      pos = strrchr( attachfileitems[i].filename, '/' ) + 1;
      // attachment will be pulled from the file named
      buffer = F("\r\n--{BOUNDARY}\r\n");
      buffer += F("Content-Type: application/octet-stream\r\n");
      buffer += F("Content-Transfer-Encoding: base64\r\n");
      buffer += F("Content-Disposition: attachment;filename=\"");
      buffer += pos ;
      buffer += F("\"\r\n");
      client->println(buffer);
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("CLIENT->SERVER: "));
  Serial.println(buffer);
  size_t bytessent = 0;
#endif
      // read data from file, base64 encode and send it
      // 3 binary bytes (57) becomes 4 base64 bytes (76)
      // plus CRLF is ideal for one line of MIME data
      // 570 bytes will be read at a time and sent as ten lines of base64 data
      uint8_t * fdata = (uint8_t*) malloc( 570 );
      // read data from file
      flen = fread( fdata, 1, 570, atfile );
      delay(10);
      buffer2 = "";
      int lc = 0;
      size_t bytecount = 0;
      while ( flen > 0 ) {
        while ( flen > 56 ) {
          // convert to base64 in 57 byte chunks
          buffer2 += b.encode( fdata+bytecount, 57 );
          // tack on CRLF
          buffer2 += "\r\n";
          bytecount += 57;
          flen -= 57;
        }
        if ( flen > 0 ) {
          // convert last set of bytes to base64
          buffer2 += b.encode( fdata+bytecount, flen );
          // tack on CRLF
          buffer2 += "\r\n";
        }
        // send the lines in buffer
        client->println( buffer2 );
#ifdef DEBUG_EMAIL_PORT
  bytessent += bytecount + flen;
  Serial.print(F(" bytes sent so far: ")); Serial.println(bytessent);
#endif
        buffer2 = "";
        bytecount = 0;
        delay(10);
        flen = fread( fdata, 1, 570, atfile );
      }
      fclose( atfile );
      free( fdata );
      fdata = NULL;
    } 
  }
#endif

  buffer = F("\r\n--{BOUNDARY}--\r\n.");
  client->println(buffer);
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("CLIENT->SERVER: "));
  Serial.println(buffer);
#endif
  buffer = F("QUIT");
  client->println(buffer);
#ifdef DEBUG_EMAIL_PORT
  Serial.print(F("CLIENT->SERVER: "));
  Serial.println(buffer);
#endif
  return true;
}

void SendEmail::close() {

  // cleanup buffer attachments
  for ( int i = 0; i < attachbuffercount; i++ ) {
    free( attachbufferitems[i].buffername );
    attachbufferitems[i].buffername = NULL;
  }

#ifdef USING_SD_CARD
  // cleanup file attachments
  for ( int i = 0; i < attachfilecount; i++ ) {
    free( attachfileitems[i].filename );
    attachfileitems[i].filename = NULL;
  }
#endif
  
  client->stop();
  delete client;

}
