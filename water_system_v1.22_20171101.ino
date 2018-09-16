
/*
   1.22 변경내역
    - Light 조명센서 추가. LIGHT_SENSOR_PIN A2
   1.21 변경내역
    - OLED 표시를 푸쉬 버튼을 누를 때만, 표시하도록 수정함. OLED_ONOFF_PIN=7
   1.2 변경내역
    - OLED 연동 추가
   1.1 변경내역
    - ESP8266을 이용한 웹서버 기능을 제거하고, thingspeak.com 으로 데이터를 전송함.
    - 전송주기 1분 단위로 데이터 전송
   1.0 개발내역
    - 시리얼통신을 이용한 ESP8266 웹서버 기능 제공. 시리얼 통신에서 긴 text 통신에서 오류가 발생해서, 단문으로 수정.
    - 토양습도 체크
    - 온도/습도 체크
    - 통양습도가 일정 수치 미만에서 릴레이를 동작 시켜, 물공급.
*/

#include <SoftwareSerial.h>
#include "DHT.h"
#include <U8g2lib.h>

#define DEBUG false

#define ARRAY_SIZE(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))
#define DHT_TYPE DHT22   // DHT11, DHT22(AM2302, AM2321)

// 핀설정
#define SOIL_SENSOR_PIN A0  // what digital pin we're connected to
#define LIGHT_SENSOR_PIN A2  // what digital pin we're connected to
#define DISPLAY_ONOFF_PIN 4 // 화면 표시 제어 핀. on 시 기간시간동안 정보를 표시한다.
#define DHT_PIN 6           // what digital pin we're connected to
#define SOIL_RELAY_PIN 8    // 토양센서에 결과에 따른 제어 relay 핀
#define SDCARD_CS_PIN 10

// 환경설정 부분
#define DHT_USE_YN 1
#define SOIL_SENSOR_USE_YN 1
#define WATER_RELAY_USE_YN 1

// 땡땡이 토마토 기준
int soilHumidityOffset = 25; // 실재 습도 보정치. 샤오미 센서랑 비교할 때 20% ~ 25%정도 습도가 높게 측정됨.
int soilHumidityIndex = 40 + soilHumidityOffset; // 물을 주기 위한 습도 임계치값. 임계치 이하면, 물을 공급한다.
// 땡땡이는 일단, 5초만 수분 공급~
int waterSupplyTime = 8; // 물 공급 시간 (기본:10초)
int waterSupplyIntervalTime = (60 * 10); // 물공급 인터벌 시간. 기본: 10 분(60초 * 10분)
unsigned long loopIntervalTime = 10; // 루프 인터벌 시간.  아두이노 우노

// loop 에서 센서 처리 여부
int checkedSensor = 0;
// 물공급 delya time
int waterDelayTime = 0;

// soil config
int soilRhMaxWhenAnalHighYn = 1; // 아나로그값이 Max일 때, 습도도 Max 값이지 여부. 센서 종류에 따라 비례하는 게 있고, 반비례하는 것도 있음.

unsigned long previousMillis = 0;        // will store last time LED was updated
//unsigned int ui1000 = 1000;

struct Humidity {
  int analogValue;
  int digitalValue;
};

// any 변수
int gSoilDigitalValue = 100;
int gSoilAnalogValue = 100;
int gLightValue = 0;
float gDhtHumidity = 0.0;
float gDhtTemperature = 0.0;
String gIp = "";
String apiKeyWriteToThingSpeak = "W27HDSSJHOAQHK3S";
// display mode (log, info, etc)
boolean gSetupMode = true;
// display button
boolean gDisplayCurrentButton = LOW;
boolean gDisplayLastButton = LOW;
unsigned long previousDisplayMillis = 0;
boolean gDisplayState = false;
int gDisplayDurationTime = 30; // 화면표시 시간(초)

String sMessage = "";

/////////////////////////////////////////////////
// ESP8266 wifi 모듈
/////////////////////////////////////////////////
SoftwareSerial esp8266(2, 3); // make RX Arduino line is pin 2, make TX Arduino line is pin 3.
// This means that you need to connect the TX line from the esp to the Arduino's pin 2
// and the RX line from the esp to the Arduino's pin 3

/////////////////////////////////////////////////
// DHT 센서
/////////////////////////////////////////////////
DHT dht(DHT_PIN, DHT_TYPE);

/////////////////////////////////////////////////
// OLED 128x64, u8g2_font_6x13_t_hebrew 폰트 화면 설정
/////////////////////////////////////////////////

#define MAX_LINES 5
int gCurLines = 0; // 현재 라인 수. 0 부터 시작하는 걸로.
int gMaxLines = MAX_LINES; // font에 따른 max line 수. -> 폰트에 따라 계산할 수도. 
String gMsgArray[MAX_LINES];
String gStrSpace = "";
u8g2_uint_t initX = 2;
u8g2_uint_t gLineGap = 12; //pixel
// OLED - SSD1306 128x64
U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display


/////////////////////////////////////////////////
// SD 카드 설정. 아두이노 UNO, Nano에는 메모리가 부족해서 적용 불가.
/////////////////////////////////////////////////
//#include <SPI.h>
//#include "SdFat.h"
//SdFat SD;
//#define SD_CS_PIN 10
//File myFile;

void setup() {
  u8g2.begin(); // OLED begin
  initScreen(); // OLED 화면 초기화
    
  Serial.begin(9600);
  esp8266.begin(9600); // your esp's baud rate might be different

  setupEsp8266();

  // setup dht
  if (DHT_USE_YN) {
    logInfo("DHT begin...");
    dht.begin();
  }

  // setup soil relay
  pinMode(SOIL_RELAY_PIN, OUTPUT);
  // Light sensor
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  // 화면표시 ON/OFF 입력
  pinMode(DISPLAY_ONOFF_PIN, INPUT); 
  
  logInfo("Water System start!");
  delay(500);
  
  displayBlankScreen();
  gSetupMode = false;
}

void loop() {

  //wifi Server 기능. thingspeak.com 으로 데이터 전송 기능 추가로 주석처리함. 2017-10-14
  //processWifiServer();

  // 화면표시 버튼을 누를 때, 화면에 정보를 일정시간동안 표시한다.
  processDisplayOnoff();

  // looping
  unsigned long currentMillis = millis();
  if (previousMillis > 0 && (currentMillis - previousMillis) < (loopIntervalTime * 1000)) { // loop 간격. wifi 만 체크하고 그외 시간은 pass 한다.
    return;
  } else {
    previousMillis = currentMillis;
  }

  ////////////////////////////////////////////
  // start check sensor
  ////////////////////////////////////////////

  sMessage = "";
  checkedSensor = 0;

  // csd 광도를 측정한다.
  processLightSensor();
  

  // DHT 센서 처리
  if (DHT_USE_YN) {
    processDhtSensor();
    checkedSensor++;
  }

  // 토양 센서 처리
  if (SOIL_SENSOR_USE_YN) {
    processSoilSensor();
    checkedSensor++;
  }

  if (DEBUG && checkedSensor > 0) {
    logInfo(getMessage());
  }
    
  // send Message to thingspeak.com
  //sendMessageToThingSpeak();
}

void processLightSensor() {
  int value = analogRead(LIGHT_SENSOR_PIN);
  gLightValue = value;
  logInfo("### gLightValue : " + String(gLightValue));
}

// DHT 센서 처리
void processDhtSensor() {
  // DHT 센서 처리 ////////////////////////////
  gDhtHumidity = dht.readHumidity();
  gDhtTemperature = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(gDhtHumidity) || isnan(gDhtTemperature)) {
    sMessage += "\tFailed to read from DHT sensor!";
    return;
  }
}

Humidity getSoilHumidity(int soilSensorPinNm) {
  Humidity humidity = {0, 0};

  int soilAnalogValue = analogRead(soilSensorPinNm);
  if (!soilRhMaxWhenAnalHighYn) {
    soilAnalogValue = 1023 - soilAnalogValue;
  }

  // Max(물에 잠길때) : 540 ~ 630, Min: 0
  // 2차 측정 : Max 760
  // 변환된 값 Max를 1023이라고 가정하고 digitalValue 계산
  int soilDigitalValue = map(soilAnalogValue, 0, 1023, 0, 100);
  soilDigitalValue = constrain(soilDigitalValue, 0, 100);

  //return soilDigitalValue;
  humidity.analogValue = soilAnalogValue;
  humidity.digitalValue = soilDigitalValue;
  return humidity;
}


void processSoilSensor() {
  // 토양 센서 처리 //////////////////////////
  Humidity humidity = getSoilHumidity(SOIL_SENSOR_PIN);
  gSoilDigitalValue = humidity.digitalValue;
  gSoilAnalogValue = humidity.analogValue;

  if (WATER_RELAY_USE_YN && (gSoilDigitalValue < soilHumidityIndex)) {
    // 습도가 0보다는 크고, 인덱스값보다 작을 때 물을 공급한다.
    if (gSoilDigitalValue < 3) {
      //sMessage += "\t토양센스가 습도를 측정하고 있지 않습니다."; // 3미만은 미측정으로 처리한다.
      sMessage += "\t토양센스 Not work"; // 3미만은 미측정으로 처리한다.
    } else {
      waterDelayTime -= loopIntervalTime;
      if (waterDelayTime < 1) { // 물공급 인터벌 시간이 지나야, 추가로 물을 공급할 수 있다.
        waterDelayTime = waterSupplyIntervalTime; // 물공급 지연 시간
        digitalWrite(SOIL_RELAY_PIN, HIGH);
        sMessage += "\trelay on(";
        sMessage += String(waterSupplyTime);
        sMessage += "초)";
        delay(waterSupplyTime * 1000);     // 물주기 시간
        digitalWrite(SOIL_RELAY_PIN, LOW);
        sMessage += "\trelay off";
      } else {
        //sMessage += "\t물공급 인터벌 시간이 아직 지나지 않았습니다.(";
        sMessage += "\t물공급 인터벌(";
        sMessage += String(waterDelayTime);
        sMessage += "초 남음)";
      }
    }
  }
}

/*
     Send Message to Thingspeak.com
     Tomato1 Channel
     field1 : Soil Moisture (%)
     field2 : Soil Moisture (Raw)
     field3 : Temperature (°C)
     field4 : Humidity (RH)
     field5 : Light
     field6 :
     field7 :
     field8 :
*/
void sendMessageToThingSpeak() {
  String connectSend = "AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n";
  sendData(connectSend, 2000, DEBUG);

  String v1 = "50";
  String v2 = "500";
  String postStr = apiKeyWriteToThingSpeak;
  postStr += "&field1=";
  postStr += String(gSoilDigitalValue);
  postStr += "&field2=";
  postStr += String(gSoilAnalogValue);
  postStr += "&field3=";
  postStr += String(gDhtTemperature);
  postStr += "&field4=";
  postStr += String(gDhtHumidity);
  postStr += "&field5=";
  postStr += String(gLightValue);
  postStr += "\r\n\r\n";

  String headerStr = "";
  headerStr += "POST /update HTTP/1.1\n";
  headerStr += "Host: api.thingspeak.com\n";
  headerStr += "Connection: close\n";
  headerStr += "X-THINGSPEAKAPIKEY: " + apiKeyWriteToThingSpeak + "\n";
  headerStr += "Content-Type: application/x-www-form-urlencoded\n";
  headerStr += "Content-Length: ";
  headerStr += postStr.length();
  headerStr += "\n\n";
  headerStr += postStr;

  // send
  String cipSend = "AT+CIPSEND=";
  cipSend += headerStr.length();
  cipSend += "\r\n";

  sendData(cipSend, 1000, DEBUG);
  sendData(headerStr, 1000, DEBUG);

  // close
  String closeCommand = "AT+CIPCLOSE";
  closeCommand += "\r\n";
  sendData(closeCommand, 3000, DEBUG);
}

String getMessage() {
  String tempMessage = "";

  if (DHT_USE_YN) {
    //tempMessage += "Humidity: ";
    tempMessage += "H: ";
    tempMessage += String(gDhtHumidity);
    tempMessage += "%\t";

    //tempMessage += "Temperature: ";
    tempMessage += "T: ";
    tempMessage += String(gDhtTemperature);
    tempMessage += "*C\t";
  }

  // 토양 센서 처리
  if (SOIL_SENSOR_USE_YN) {
    //tempMessage += "Soil moisture(Digital/Analog): ";
    tempMessage += "Soil(D/A): ";
    tempMessage += String(gSoilDigitalValue);
    tempMessage += " (%) / ";
    tempMessage += String(gSoilAnalogValue);
    tempMessage += " ";
  }

  tempMessage += sMessage;
  if (tempMessage.length() < 1) {
    tempMessage = "No Message";
  }

  return tempMessage;
}

/*
void processWifiServer() {
  if (esp8266.available() > 6) { // check if the esp is sending a message, 6byte는 받아야 connectionId가 포함됨.
    if (esp8266.find("+IPD,")) {
      // connectionId 정보가 도착할 때까지 지연시킴. 매우.... 느림.
      // delay없으면, -1 값이 출력됨. 성공: 20, 30, 50, 100 / 실패: 10
      delay(100);

      int intConnectionId = esp8266.read() - 48; // subtract 48 because the read() function returns
      // the ASCII decimal value and 0 (the first decimal number) starts at 48
      String connectionId = String(intConnectionId);

      String response = "";
      char c = 0;
      while (esp8266.available() > 0) {
        // The esp has data so display its output to the serial window
        c = esp8266.read(); // read the next character.
        response += c;
      }

      //     if(DEBUG && response.length() > 0) {
      //      Serial.println("\r\n------------------ processWifi read start");
      //      Serial.println(response);
      //      Serial.println("------------------ processWifi read end");
      //     }

      sendWebPage(connectionId);
      closeHtml(connectionId);
    }
  }
}
*/

/*
   시리얼 통신을 이용한 경우, 전송 text 길이가 길면 오류가 잦음.
   작은 용량 정도만 가능하고, 외부 서버나 esp8266에서 자체 서버를 이용해야 함.
   text를 나눠서, head 전송 후, 나눠 운용이 가능하면 좋은데... 개선이 필요함.
*/
/*
void sendWebPage(String connectionId) {

  String message = getMessage();

  String webpage = "";
  //    webpage  = "HTTP/1.1 200 OK\r\n";
  //    webpage += "Content-Type: text/html\r\n";
  //    webpage += "Connection: close\r\n";
  //    webpage += "Refresh: 10\r\n";
  //    webpage += "\r\n";

  //    sendHtml(connectionId, webpage);

  //    webpage += "<!DOCTYPE HTML>";
  webpage += "<html><head>";
  webpage += "<meta charset=\"UTF-8\">";
  webpage += "<link rel=\"icon\" href=\"data:,\">";
  webpage += "</head><body>";
  //    webpage += "<h1>Hello</h1><h2>";
  webpage += message;
  //    webpage += "</h2><button>LED1</button><button>LED2</button>";
  webpage += "</body></html>";

  // 잘 동자하지 않음. header 전송 후, link 수신 받고, 추가 전송을 어떻게 처리해야 하는지 자료를 찾아야 함. 2017-10-09
  //sendHtml(connectionId, webpage);


  String cipSend = "AT+CIPSEND=";
  cipSend += connectionId;
  cipSend += ",";
  cipSend += webpage.length();
  cipSend += "\r\n";

  sendData(cipSend, 1000, DEBUG);
  sendData(webpage, 1000, DEBUG);

}
*/

/*
String sendHtml(String connectionId, String webpage) {
  String cipSend = "AT+CIPSEND=";
  cipSend += connectionId;
  cipSend += ",";
  cipSend += webpage.length();
  cipSend += "\r\n";

  sendData(cipSend, 1000, DEBUG); // cipsend - maxlength 2048
  sendData(webpage, 1000, DEBUG);
}
*/

/*
void closeHtml(String connectionId) {
  String closeCommand = "AT+CIPCLOSE=";
  closeCommand += connectionId; // append connection id
  closeCommand += "\r\n";
  sendData(closeCommand, 3000, DEBUG);
}
*/

void setupEsp8266() {
  logInfo("ESP8266 ESP-01 module");

  int cwmode = 1;
  sendData("AT+RST\r\n", 2000, DEBUG); // reset module
  sendData("AT+CWMODE=" + String(cwmode) + "\r\n", 1000, DEBUG); // configure as access point
  sendData("AT+CWJAP=\"U+Net5B6F\",\"1000019118\"\r\n", 5000, DEBUG);
  sendData("AT+CIFSR\r\n", 1000, DEBUG); // get ip address

  if (cwmode != 1) {
    sendData("AT+CWSAP=\"esp8266_j2h\",\"12345678\",5,3\r\n", 1000, DEBUG); // ap모드
  }

  // 클라이언트로 사용
  sendData("AT+CIPMUX=0\r\n", 1000, DEBUG); // configure for multiple connections
  
// 서버모드로 사용  
//  sendData("AT+CIPMUX=1\r\n", 1000, DEBUG); // configure for multiple connections
//  sendData("AT+CIPSERVER=1,8080\r\n", 1000, DEBUG); // turn on server on port 80
}

/*
   timeout 최소 20ms 은 기다려야 함.
*/
String sendData(String command, const int timeout, boolean debug) {
  /*
    if(debug && command.length() > 0) {
    Serial.println("\r\n\r\n>>>>>>>>>>>>>>>>>>>> sendData() send start");
    Serial.println(command);
    }
  */
  if (esp8266.overflow()) {
    clearEsp8266();
  }

  String response = "";
  esp8266.print(command); // send the read character to the esp8266
  long int time = millis();

  while ( (time + timeout) > millis()) {
    while (esp8266.available() > 0) {
      // The esp has data so display its output to the serial window
      char c = esp8266.read(); // read the next character.
      response += c;
    }
  }

  if(command.indexOf("AT+CIFSR") > -1) {
    gIp = getIp(response);
  }

  if (gSetupMode && response.length() > 0) {
    logInfo("\r\n<<<<<<<<<<<<<<<<<<<< sendData() read start");
    logInfo(response);
  }

  return response;
}

void clearEsp8266() {
  long int time = millis();
  int timeout = 20;
  while ( (time + timeout) > millis()) {
    while (esp8266.available()) {
      while (esp8266.read() != -1) {}
    }
  }
}

String getIp(String message) {
  int count = 0;
  int index = 0;
  String ip = "";
  
  index = message.indexOf("\n");
  while(index > -1) {
    count++;
    index = message.indexOf("\n", index+1);
  }

  String msgArray[count+1];
  count = 0;
  int preIdx = 0;
  int postIdx = message.indexOf("\n"); 
  while(postIdx > -1) {
    preIdx = postIdx;
    postIdx = message.indexOf("\n", preIdx+1);
    msgArray[count] = message.substring(preIdx+1, postIdx);
    msgArray[count].trim();
    count++;
   }

   msgArray[count] = message.substring(preIdx); //.trim();    
   msgArray[count].trim();

  int arrCnt = ARRAY_SIZE(msgArray);
  for(int i = (arrCnt - 1); i > -1; i--) {
    if(msgArray[i].length() < 1) {
      continue;
    }

    if(isDigit(msgArray[i].charAt(0))) {
      ip = msgArray[i];
      break;
    }
  }

  return ip;
}

void logInfo(String message) {
  if(DEBUG) {
    Serial.println(message);  
    Serial.flush();    
  }
  
  if(gDisplayState || gSetupMode) {
    logScreen(message);    
  }
}

//////////////////////////////
// screen
//////////////////////////////
void processDisplayOnoff() {
  unsigned long currentMillis = millis();
  gDisplayCurrentButton = getDisplayButtonState(gDisplayLastButton);
//  logInfo("gDisplayCurrentButton:" + String(gDisplayCurrentButton)); // 풀다운 on/off 체크
  if(gDisplayCurrentButton == HIGH) {
      gDisplayState = true;
      previousDisplayMillis = millis();
  }
  gDisplayLastButton = gDisplayCurrentButton;

  if(gDisplayState) {
    if ((currentMillis - previousDisplayMillis) < (gDisplayDurationTime * 1000)) { // loop 간격. wifi 만 체크하고 그외 시간은 pass 한다.
      displayInfoScreen();
    } else {
      displayBlankScreen();
      gDisplayState = false;
    }
  }  
}

boolean getDisplayButtonState(boolean push) {
  boolean current = digitalRead(DISPLAY_ONOFF_PIN);
  if(push != current) { // 채터링 방지 로직
      delay(5);
      current = digitalRead(DISPLAY_ONOFF_PIN);
  }
  return current;
}

void displayBlankScreen() {
  u8g2.clear();
}

void displayInfoScreen() {
  u8g2.firstPage();
  do {
    u8g2.drawStr(2, 12, ("IP: " + gIp).c_str());
    u8g2.drawLine(0, 14, 128, 14);
    u8g2.drawStr(2, 26, ("Soil Moisture: "+String(gSoilDigitalValue)+"%").c_str());
    u8g2.drawStr(2, 38, ("Temperature  : "+String(gDhtTemperature,1)+"'C ").c_str());
    u8g2.drawStr(2, 50, ("Humidity     : "+String(gDhtHumidity,1)+"%").c_str());
    u8g2.drawStr(2, 62, ("Light        : "+String(gLightValue)).c_str());
  } while ( u8g2.nextPage() );
}

void initScreen() {
  u8g2.setContrast(0);         
  u8g2.setFont(u8g2_font_6x13_t_hebrew);
  //u8g2.setFontMode(0);  
}

void logScreen(String tempMessage) {
  int arrySize = 1;
  int index = tempMessage.indexOf("\n");
  while(index > -1) {
    arrySize++;
    index = tempMessage.indexOf("\n", index+1);
  }

  String msgArray[arrySize];
  int count = 0;
  int preIdx = -1;
  int postIdx = tempMessage.indexOf("\n"); 
  while(postIdx > -1) {
    msgArray[count] = tempMessage.substring(preIdx+1, postIdx);
    msgArray[count].trim();
    count++;    
    preIdx = postIdx;
    postIdx = tempMessage.indexOf("\n", preIdx+1);
   }
   
   msgArray[count] = tempMessage.substring(preIdx); //.trim();    
   msgArray[count].trim();  

   for(int i = 0; i < arrySize; i++) {
    if(msgArray[i].length() < 1) {
      continue;
    }
    logLineScreen(msgArray[i]);
   }
}

void logLineScreen(String tempMessage) {
  if(gCurLines < gMaxLines) {
    gMsgArray[gCurLines++] = tempMessage;    
    // 나머지 라인 초기화
    for(int i = gCurLines; i < gMaxLines; i++) {
      gMsgArray[i] = gStrSpace;
    }  
  } else {
    // 라인 스크롤
    for(int i = 1; i < gMaxLines; i++) { 
      gMsgArray[i-1] = gMsgArray[i];
    }
    // 마지막 라인 설정
    gMsgArray[gMaxLines-1] = tempMessage;
  } 

  displayLogScreen();
}

void displayLogScreen() {
  u8g2_uint_t lines[gMaxLines];
  for(int i = 0; i < gMaxLines; i++) {
    lines[i] = gLineGap * (i + 1);
  }

  char* msg[5];
  for(int i = 0; i < gMaxLines; i++) {
    msg[i] = gMsgArray[i].c_str();
  }
  
  u8g2.firstPage();
  do {    
    for(int i = 0; i < gMaxLines; i++) {
      u8g2.drawStr(initX, lines[i], msg[i]);      
    }
  } while ( u8g2.nextPage() );
}
