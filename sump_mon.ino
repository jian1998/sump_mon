#include "esp_camera.h"
#include <WiFi.h>
#include <EEPROM.h>
#include "my_config.h"
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>   // Universal Telegram Bot Library written by Brian Lough: https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot
#include <ArduinoJson.h>
#include <ArduinoOTA.h>

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15 
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well


#include "camera_pins.h"

#define MAX_GAIN                      GAINCEILING_8X    // GAINCEILING_4X  or GAINCEILING_16X
#define MAGIC_WORD                    ((int32_t)0x56BAAB65)
#define MAGIC_LOC                     500   // EEPROM address of the MAGIC_WORD
#define NOMINAL_PUMP_TIME_LOC         508   // EEPROM address of the nominal pump on time  
#define TOTAL_EEPROM_REC              30    // 6 x 30 = 180 bytes + 4 bytes(MAGIC_WORD) = 184 bytes total out of the 512 bytes of EEPROM
#define AVG_WINDOW                    5     // number of last records for average pump on time calculation
#define NOMINAL_COUNTS                (AVG_WINDOW + 2)    // use the first NOMINAL_COUNTS pump on durations to calculate nominal pump on duration value

#define ALARM_SEND_INTERVAL           120   // now often to send alarm messages (sec.)
#define EVENT_FREQ_CHANGE_THRESHOLD   0.5   // freq of last event deviated from the average by 50% 
#define DURATION_CHANGE_THRESHOLD     0.2   // last pump on duration deviated from the initial averaged duration by 20%
#define MIN_PERIOD_INTERVAL           10.   // minimum event interval (more freq than this interval will cause alarm.

   
#define FLASH_LED_PIN   GPIO_NUM_4
#define PUMP_CT_PIN     GPIO_NUM_15
#define WATER_LEVEL_PIN GPIO_NUM_13
#define BUILT_IN_LED    GPIO_NUM_33
#define LINE_OK_PIN     GPIO_NUM_1
#define USE_BAT_PIN     GPIO_NUM_3


void startCameraServer();

WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

bool isWebConnected = false;

float nominal_pump_on_time  = 0.;
float last_on_time          = 0.;
float avg_period            = 0.;
float since_last            = 0.;
bool  pump_on_z             = false;
bool  system_ok             = true;
bool link_ok                = true;
int   new_pump_event_idx    = -1;  // -1 for no new data, yet
time_t system_restart_time = 0;

int next_pump_record = 0;   // where in the pump_records to add the new pump event

struct pump_record_t
{
    int32_t   event_time;  
    uint16_t  duration;
}pump_records[TOTAL_EEPROM_REC]; 

void send_teletram_msg(char * msg)
{
  bot.sendMessage(CHAT_ID, msg, "");  
}

void reset_pump_history()
{
    EEPROM.writeInt(MAGIC_LOC, 0x0);  // blank out the magic word so it will clear the history at next boot
    EEPROM.commit();
    delay(100);
    ESP.restart();
}

void turn_flash_on(bool isOn)
{
    digitalWrite(FLASH_LED_PIN, isOn);
}


void get_saved_sump_events()
{
  int i;
  int32_t magic = EEPROM.readInt(MAGIC_LOC); // read MAGIC word
  if(magic != MAGIC_WORD)
  {
//    Serial.println(String("MAGIC_WORD from EEPROM ") + magic);
//    Serial.println(" **** No MAGIC_WORD, Clear EEPROM **** ");
    // clear out all records 
    for (i = 0; i < 64; i++)
    {
      EEPROM.writeLong64(i*8, 0x0); // set all 512 bytes to 0x0.
    }      
    EEPROM.writeInt(MAGIC_LOC, MAGIC_WORD);
    EEPROM.writeInt(NOMINAL_PUMP_TIME_LOC, 0x0);
    EEPROM.commit();
//    Serial.println(" End Clean EEPROM "); 
  }
//  Serial.println("start to read EEPROM records");
  bool found_next_loc = false;  
  pump_records[0].event_time = EEPROM.readLong(0);
  pump_records[0].duration   = EEPROM.readShort(4);
  if (pump_records[0].event_time == 0)
  {
    // there is no events in the EEPROM. 
    found_next_loc = true;
    next_pump_record = 0;
  }
  for (i = 1; i < TOTAL_EEPROM_REC; i++)
  {

    pump_records[i].event_time = EEPROM.readLong(i*6);
    pump_records[i].duration   = EEPROM.readShort(i*6+4);

    if(pump_records[i].event_time < pump_records[i -1].event_time) 
    {
      // this event happened earlier than the previous event, 
      // this is where we can overwrite for the next event
      found_next_loc = true;
      next_pump_record = i;
    }
  }
  nominal_pump_on_time = EEPROM.readInt(NOMINAL_PUMP_TIME_LOC);
    
  if(!found_next_loc) 
  next_pump_record = 0;

  char buf[100];
  char time_output[30];

  for (i = 0; i < TOTAL_EEPROM_REC; i++)
  {
    time_t tm = pump_records[i].event_time;
    if (tm > 1000)
    {
      strftime(time_output, 30, "%D %T", localtime(&tm)); 
      sprintf(buf, "%02i) \"%s\",%i\n", i ,time_output, pump_records[i].duration);
//      Serial.print(buf);
    }
  }
//  Serial.println(String("next_pump_record: ") + next_pump_record); 
  int idx = next_pump_record -1;
  if(idx < 0) idx += TOTAL_EEPROM_REC;
  int lastTime = pump_records[idx].event_time;
  idx = idx -AVG_WINDOW;  
  if(idx < 0) idx += TOTAL_EEPROM_REC;
  int timeInterval = lastTime - pump_records[idx].event_time;
  avg_period = (float)timeInterval / (float) AVG_WINDOW / 60.;

}  

//-------------------------------------------------------
// get sump history data for chart in the web interface 
//-------------------------------------------------------
bool get_init_chart_data(char * response)
{
  char buf[100];
  char time_output[30];      
  strcpy(response,"{\"responseText\": [");
  for (int i = 0; i < TOTAL_EEPROM_REC; i++)
  {
    if(pump_records[i].event_time == 0) 
        break;
    
    if(i > 0)strcat(response,",");
    if(pump_records[i].event_time > 0) 
    {
      time_t tm = pump_records[i].event_time;
     
      strcat(response,"[");
      strftime(time_output, 30, "%D %T", localtime(&tm)); 
      sprintf(buf, "\"%s\",%i",time_output, pump_records[i].duration);
      strcat(response,buf);
      strcat(response,"]\n");
    }
  }
  strcat(response,"]}");
  return true;
  
}

bool get_new_chart_data(char * response)
{
  
  char buf[100];
  char time_output[30];      
  time_t now;
  time(&now);

  strcpy(response,"{\"responseText\": ");
  strcat(response,"[");  
  if (new_pump_event_idx >= 0)
  {
    time_t tm = pump_records[new_pump_event_idx].event_time;
    strftime(time_output, 30, "%D %T", localtime(&tm)); 
    sprintf(buf, "\"%s\",%i",time_output, pump_records[new_pump_event_idx].duration);
    new_pump_event_idx = -1;
  }
  else
  {
    strftime(time_output, 30, "%D %T", localtime(&now)); 
    sprintf(buf, "\"%s\", null",time_output);
  }
  strcat(response,buf);
  strcat(response,"]\n");
  strcat(response,"}");
  return true;
}

void setup_camera()
{
  
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
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_XGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      config.frame_size = FRAMESIZE_XGA;
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_XGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
//    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  s->set_vflip(s, 1); // flip it back
  s->set_hmirror(s, 1);
  s->set_brightness(s, 4); // up the brightness just a bit
  s->set_saturation(s, -2); // lower the saturation
  // drop down frame size for higher initial frame rate
  if(config.pixel_format == PIXFORMAT_JPEG){
    s->set_framesize(s, FRAMESIZE_SXGA);
  }

  s->set_gainceiling(s,MAX_GAIN);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

}

void setup_wifi()
{
  IPAddress local_IP;
  local_IP.fromString(cam_ip);
  IPAddress gateway;
  gateway.fromString(cam_gateway);
  IPAddress primaryDNS;
  primaryDNS.fromString(cam_gateway);
  // Following three settings are optional
  IPAddress subnet(255, 255, 255, 0);
  IPAddress secondaryDNS(8, 8, 4, 4);

  
  WiFi.setHostname(CAM_NAME);
  WiFi.mode(WIFI_STA);   // to disable the access point mode
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
   // This part of code will try create static IP address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) 
  {
//    Serial.println("STA Failed to configure");
  }
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  int retry_cnt = 0;
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
//    Serial.print(".");
    if(++retry_cnt > 30) ESP.restart();
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}
void setup_OTA()
{
 
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(CAM_NAME);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
  // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
//    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
//    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
//    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));

  });
/*  
  ArduinoOTA.onError([](ota_error_t error) {

   Serial.printf("Error[%u]: ", error);

    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
 */
  ArduinoOTA.begin();
}

void check_wifi_connection()
{
  static unsigned long previousMillis = 0;
  const unsigned long interval = 120000;    // re-connect if lost connection over 2 min.
  unsigned long currentMillis = millis();
  // if WiFi is down, try reconnecting
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
 //   Serial.print(millis());
 //   Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }
}


void setup() 
{
//  Serial.begin(115200);
//  Serial.setDebugOutput(true);    // somehow this will cause the EEPROM.commit() to crash the application

  setup_camera();
  setup_wifi();
  setup_OTA();

  secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT);    // needed for telegram bot

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  setenv("TZ", my_timezone, 1); 
  
  startCameraServer();

  pinMode(FLASH_LED_PIN, OUTPUT);
  pinMode(PUMP_CT_PIN, INPUT_PULLUP);
  pinMode(BUILT_IN_LED, OUTPUT);
  pinMode(LINE_OK_PIN, INPUT);
  pinMode(USE_BAT_PIN, OUTPUT);
  pinMode(WATER_LEVEL_PIN,INPUT_PULLUP);
  
  EEPROM.begin(512);
  get_saved_sump_events();


}


void loop() 
{
  check_wifi_connection();
  ArduinoOTA.handle();


  // turn on flashlibht right when web server is serving 
  static long lastCaptureReqTime = 0;

  static time_t pump_on_time = 0;
  static bool alarm_from_event = false;  
  static bool alarm_from_update = false;  
  bool pump_on = digitalRead(PUMP_CT_PIN);

  static bool line_ok_z = true;
  bool level_limit = digitalRead(WATER_LEVEL_PIN);
  time_t now;
  static time_t last_alarm_msg_sent = 0;
  String alarm_msg;
  int alarm = 0;
  char buf[100];
  static int time_stable_cnt = 0;
  

  link_ok = digitalRead(LINE_OK_PIN);
 
  if(link_ok) 
  {
    digitalWrite(USE_BAT_PIN, 1);  // use ac line power
    if(link_ok != line_ok_z)
    {
      bot.sendMessage(CHAT_ID, "sump pump back to line power\n", "");     
    }

  }
  else
  {
    if(level_limit || pump_on)
    {
        digitalWrite(USE_BAT_PIN, 0);  // start using battery power
    }
    else
    {
      // still no power but pump has topped. We can turn off battery power until water level reached.
      digitalWrite(USE_BAT_PIN, 1);  // use ac line power
    }

    if(link_ok != line_ok_z)
    {
      bot.sendMessage(CHAT_ID, "sump pump is on battery power\n", "");     
    }
  }
  line_ok_z = link_ok; 
  
  time(&now);

  if(now > 100000 && system_restart_time == 0 )
  {
    // the time received right after reboot is not accurate. Wait a bit.
    if(++time_stable_cnt > 10)
       system_restart_time = now;    
  }

  
  if (pump_on != pump_on_z)
  {    
    pump_on_z = pump_on;
    if(pump_on)
    {
       // pump just turned on
       time(&pump_on_time);
    }
    else
    {
      if(now > 100000 && pump_on_time > 100000)
      {   // when system just start, it may not be able to get the time right away. Ignore those points.
        char time_output[30];
        pump_records[next_pump_record].event_time = now;
        pump_records[next_pump_record].duration = now - pump_on_time;
        new_pump_event_idx = next_pump_record;
        EEPROM.writeLong(next_pump_record * 6,  pump_records[next_pump_record].event_time);
        EEPROM.writeShort(next_pump_record * 6 +4, pump_records[next_pump_record].duration);
        EEPROM.commit();
         if ((next_pump_record == NOMINAL_COUNTS) && (nominal_pump_on_time == 0))
        {
          int sum = 0;        
          for(int i = 1; i <= NOMINAL_COUNTS; i++)
          {
            sum += pump_records[i].duration;
          }
          nominal_pump_on_time = sum / NOMINAL_COUNTS;
          EEPROM.writeInt(NOMINAL_PUMP_TIME_LOC, nominal_pump_on_time);
          EEPROM.commit();
        }


        // Start monintoring the period and duration after NOMINAL_COUNTS amount of events.
        if (nominal_pump_on_time > 0)
        {
          int idx = next_pump_record -1;
          if(idx < 0) idx += TOTAL_EEPROM_REC;
          float last_period = (float)(now - pump_records[idx].event_time) / 60.;
          
          if(((fabs(last_period - avg_period) / avg_period) > EVENT_FREQ_CHANGE_THRESHOLD) || (last_period < MIN_PERIOD_INTERVAL))
          {
              sprintf(buf,"Sump pump irregular event. last period = %.1f (min), avg period= %.1f (min)\n", last_period,avg_period );
              alarm_msg = buf; 
              alarm = 1;
          }
          float duration_ratio = fabs((float)pump_records[next_pump_record].duration - nominal_pump_on_time) / nominal_pump_on_time;
//         sprintf(buf,"duration_ratio = %f", duration_ratio);
//         Serial.println(buf);

          if(duration_ratio > DURATION_CHANGE_THRESHOLD)
          {
              sprintf(buf,"pump turned on = %i (sec.) while nominal is %.1f (sec.) \n", pump_records[next_pump_record].duration, nominal_pump_on_time);
              alarm_msg += buf; 
              alarm = 1;
          }
          if(alarm)
          {
            alarm_from_event = true;
            bot.sendMessage(CHAT_ID, alarm_msg, "");     
          }
          else
            alarm_from_event = false;
        }
        // calculate the average period of the AVG_WINDOW events
        int idx = next_pump_record -AVG_WINDOW;
        if(idx < 0) idx += TOTAL_EEPROM_REC;
        int timeInterval = pump_records[next_pump_record].event_time - pump_records[idx].event_time;
        avg_period = (float)timeInterval / (float) AVG_WINDOW / 60.;

        next_pump_record = (next_pump_record + 1) % TOTAL_EEPROM_REC;
        
        // See http://www.cplusplus.com/reference/ctime/strftime/ for strftime functions
        //strftime(time_output, 30, "%D %T", localtime(&now)); 
        strftime(time_output, 30, "%D %T", localtime(&now)); 
        sprintf(buf, "\"%s\",%i\n",time_output, now - pump_on_time);   //\n is needed for file 
//        Serial.print(String("appended: ") +  buf);
  
      }

//      Serial.println(String("next_pump_record = ") + next_pump_record);

    }
  }

  int idx = next_pump_record - 1;
  if(idx < 0) idx += TOTAL_EEPROM_REC;
  since_last = (float)(now - pump_records[idx].event_time) / 60.;

  alarm_msg = "";
  if (nominal_pump_on_time > 0 && avg_period >0)
  {
    if((((since_last - avg_period) / avg_period )> EVENT_FREQ_CHANGE_THRESHOLD) && (now - last_alarm_msg_sent) > ALARM_SEND_INTERVAL)
    {
      sprintf(buf, "Average period is %.1f (min.). It's already %i\n",avg_period, since_last);  
      alarm_msg = buf; 
      alarm_from_update = 1;
    }
    else if(pump_on && (float)((now - pump_on_time) - nominal_pump_on_time)/ nominal_pump_on_time > DURATION_CHANGE_THRESHOLD)
    {
      sprintf(buf, "Average pump on time is %.1f (sec). It's already %i (sec) and still on\n",nominal_pump_on_time, (now - pump_on_time));  
      alarm_msg = + buf; 
      alarm_from_update = 1;
    }
    else
    {
      alarm_from_update = 0;
    }
    
  }


  if(alarm_from_update && (now -  last_alarm_msg_sent ) > ALARM_SEND_INTERVAL)
  {
      last_alarm_msg_sent = now;
      bot.sendMessage(CHAT_ID, alarm_msg, "");       // Send Telegram alarm message 
  }

  if(alarm_from_update || alarm_from_event)
    system_ok = false;
  else
    system_ok = true;
  
  // isWebConnected is set every time web interface request a camera capture (every 2 sec. with ajax) 
  // if it is not set for over 10 seconds, we'll turn the flash light off 
  // not matter it was commended to be turned on or off from the web interface
  if(isWebConnected)
  {
    lastCaptureReqTime = millis();
    isWebConnected = false;
  }
  if((millis() - lastCaptureReqTime) > 10000)
  {
    turn_flash_on(false);  
  }

  static bool blinker = true;

  blinker = !blinker;
  digitalWrite(BUILT_IN_LED, blinker);
  delay(1000);  

}
