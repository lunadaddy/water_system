
/*
 ----------------------------------------------------------------------------------------------------------------
   앞으로 할일
   1. 습도 센서가 망가져서 물을 잘못 주는 경우가 있음.
     => 하루 물을 주는 Max 회수 정할 것.
     => 물주기 Max 되면, 카톡하자.
     => 물을 줄 때 카톡할 것.
   2. 카톡 연동
    - 습도가 너무 높을 때 (80% 이상)
    - 온도가 너무 낮을 때 (4도 미만)
   3. Soil Sensor 2번과, relay 2번 추가

 ----------------------------------------------------------------------------------------------------------------
   1.23 변경내역
    - 센서 측정 시 buffer 기능. 10개 측정 후 평균값 사용함. 
     . 초기화(0으로 초기화)/적재완료(setup에서 값 반복 적재)/값리턴(loop에서 측정 후, FIFO로 값 밀고, 평균값 리턴)
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

// 디버깅 여부. false 이면 serial 출력을 하지 않는다.
#define DEBUG true
String gVersion = "1.23";

//////////////////////////////////////////////////////////////////////////////////////
// 환경설정 부분
//////////////////////////////////////////////////////////////////////////////////////

// 센서 사용여부 //////////////////////////////////////////////
#define WIFI_USE_YN            1 // wifi 사용여부
#define SEND_MSG_THINGSPEAK_YN 0 // thinkspeak.com 으로 자료 전송여부
#define DHT_USE_YN 1             // DHT 온,습도 센서 사용여부
#define LIGHT_SENSOR_USE_YN 1    // 조도센서 사용여부
#define SOIL_SENSOR_USE_YN 1     // 토양센서 사용여부
#define WATER_RELAY_USE_YN 0     // 물공급 릴레이 사용여부

// 토양센서 습도에 따라 값이 High로 비례하는지, 반비레 하는지 여부. 1이면 비례, 0이면 반비례. 즉 1이면 습도가 높을 때 수치도 높고, 0이면 습도가 높을 때 수치가 낮음.
int soilRhMaxWhenAnalHighYn = 1; // 아나로그값이 Max일 때, 습도도 Max 값이지 여부. 센서 종류에 따라 비례하는 게 있고, 반비례하는 것도 있음.

String apiKeyWriteToThingSpeak = "W27HDSSJHOAQHK3S"; // thingspeak.com 연계하는 api key

// 땡땡이 토마토를 위한 수치 임. 식물에 따라 조정할 필요 있음.
int soilHumidityOffset = 25;                     // 실재 습도 보정치. 샤오미 센서랑 비교할 때 20% ~ 25%정도 습도가 높게 측정됨.
int soilHumidityIndex = 40 + soilHumidityOffset; // 물을 주기 위한 습도 임계치값. 임계치 이하면, 물을 공급한다. 40% 기준으로 보정치(soilHumidityOffset) 가산함. 

// 땡땡이는 일단, 5초만 수분 공급
int waterSupplyTime = 8;                 // 물 공급 시간 (기본:10초)
int waterSupplyIntervalTime = (60 * 10); // 물공급 인터벌 시간. 기본: 10 분(60초 * 10분)
unsigned long loopIntervalTime = 10;     // 루프 인터벌 시간.  아두이노 우노

#define DHT_TYPE DHT22                   // DHT11, DHT22(AM2302, AM2321), 온습도 센서 종류 설정

//////////////////////////////////////////////////////////////////////////////////////
// 핀설정
//////////////////////////////////////////////////////////////////////////////////////
// 아나로그핀
#define SOIL_SENSOR_PIN1 A0  // what digital pin we're connected to
#define SOIL_SENSOR_PIN2 A1  // what digital pin we're connected to
#define LIGHT_SENSOR_PIN A2  // what digital pin we're connected to
// PIN A3 // n/a
// PIN A4 // SDA pin for I2C(TWI) (reserved)
// PIN A5 // SCL pin for I2C(TWI) (reserved)

// 디지털핀
// PIN 0 // RX for Serial (reserved)
// PIN 1 // TX for Serial (reserved)
#define RX_PIN_FOR_ESP8266 2 // Rx <-> Tx of esp8266
#define TX_PIN_FOR_ESP8266 3 // Tx <-> Rx of esp8266
#define DISPLAY_ONOFF_PIN 4 // 화면 표시 제어 핀. on 시 기간시간동안 정보를 표시한다.
// PIN 5 // n/a
#define DHT_PIN 6           // what digital pin we're connected to
// PIN 7 // n/a
#define SOIL_RELAY_PIN1 8    // 토양센서에 결과에 따른 제어 relay 핀
#define SOIL_RELAY_PIN2 9    // 토양센서에 결과에 따른 제어 relay 핀
#define SDCARD_CS_PIN 10 // CS(SS) pin for SPI
// PIN 11 // MOSI pin for SPI (reserved)
// PIN 12 // MISO pin for SPI (reserved)
// PIN 13 // SCK  pin for SPI (reserved)

//////////////////////////////////////////////////////////////////////////////////////
// 함수형. 변경 필요 없음.
//////////////////////////////////////////////////////////////////////////////////////
#define ARRAY_SIZE(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

//////////////////////////////////////////////////////////////////////////////////////
// 전역 변수. 값을 변경할 필요없음. 프로그램에서 공통으로 사용하는 전역 변수임.
//////////////////////////////////////////////////////////////////////////////////////

String gIp = ""; // network ip 정보
String sMessage = ""; // 로그 메시지

#define SENSOR_BUFFER_SIZE 10 // 센서버퍼크기
int gSensorBufferSize = SENSOR_BUFFER_SIZE; 
int checkedSensor = 0; // loop 에서 센서 처리 여부
int waterDelayTime = 0; // 물공급 delya time
unsigned long previousMillis = 0;        // will store last time LED was updated

// 토양습도센서 변수
boolean gInitSoilSensorYn = false;
float gSoilDigitalArr[SENSOR_BUFFER_SIZE];
float gSoilAnalogArr[SENSOR_BUFFER_SIZE];
float gSoilDigitalValue = 100.0;
float gSoilAnalogValue = 100.0;

// Dht 온습도센서 변수
boolean gInitDhtSensorYn = false;
float gDhtHumidityArr[SENSOR_BUFFER_SIZE];
float gDhtHumidity = 0.0;
float gDhtTemperatureArr[SENSOR_BUFFER_SIZE];
float gDhtTemperature = 0.0;

// cDs 조도센서 변수
boolean gInitLightSensorYn = false;
float gLightValueArr[SENSOR_BUFFER_SIZE];
float gLightValue = 0.0;

// display mode (log, info, etc)
boolean gSetupMode = true; // setup 상태인지, 아닌지... display를 setup에서 표시하고, 이후 요청시에만 표시하기 위해 사용.
boolean gDisplayState = true; // true면, 켜진상태에서 시작되고, diplay 표시 여부를 관리한다. on 일때 duration 만큼만 표시한다.
int gDisplayDurationTime = 30; // 화면표시 시간(초)

// display button
boolean gDisplayCurrentButton = LOW;
boolean gDisplayLastButton = LOW;
unsigned long previousDisplayMillis = 0;

// OLED 128x64, u8g2_font_6x13_t_hebrew 폰트 화면 설정
#define MAX_LINES 5
int gCurLines = 0; // 현재 라인 수. 0 부터 시작하는 걸로.
int gMaxLines = MAX_LINES; // font에 따른 max line 수. -> 폰트에 따라 계산할 수도. 
String gMsgArray[MAX_LINES];
String gStrSpace = "";
u8g2_uint_t initX = 2;
u8g2_uint_t gLineGap = 12; //pixel

//////////////////////////////////////////////////////////////////////////////////////
// 센서 객체 선언.
//////////////////////////////////////////////////////////////////////////////////////

SoftwareSerial esp8266(RX_PIN_FOR_ESP8266, TX_PIN_FOR_ESP8266); // ESP8266 wifi 모듈. make RX Arduino line is pin 2, make TX Arduino line is pin 3.
DHT dht(DHT_PIN, DHT_TYPE); // DHT 센서
U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, SCL, SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display

/*
// SD 카드 설정. 아두이노 UNO, Nano에는 메모리가 부족해서 적용 불가.
#include <SPI.h>
#include "SdFat.h"
SdFat SD;
#define SD_CS_PIN 10
File myFile;
*/

void setup() {
  u8g2.begin(); // OLED begin
  initScreen(); // OLED 화면 초기화    
  // 화면표시 ON/OFF 입력
  pinMode(DISPLAY_ONOFF_PIN, INPUT); 

  Serial.begin(9600);
  logInfo("setup() - start");

  if(WIFI_USE_YN) {
    esp8266.begin(9600); // your esp's baud rate might be different
    setupEsp8266();    
  }

  // Light sensor
  if(LIGHT_SENSOR_USE_YN) {
    pinMode(LIGHT_SENSOR_PIN, INPUT);
    logInfo("Light begin");
    initLightSensor();
  }
  
  // setup dht
  if (DHT_USE_YN) {
    dht.begin();
    logInfo("DHT begin");
    initDhtSensor();
  }
    
  // setup soil relay
  if(SOIL_SENSOR_USE_YN) {
    pinMode(SOIL_RELAY_PIN1, OUTPUT);
    logInfo("Soil Sensor begin");
    initSoilSensor();  
  }

  logInfo("setup() - end");  
  logInfo("Water System start!");
  delay(500);
  
  displayBlankScreen();
  gSetupMode = false;
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
  if(LIGHT_SENSOR_USE_YN) {
    processLightSensor();    
  }
  
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
  if(WIFI_USE_YN && SEND_MSG_THINGSPEAK_YN) {
    sendMessageToThingSpeak();    
  }
}

void initLightSensor() {
  // Soil Sensor 초기화. 정상수치 나오는데, 5초 정도 걸림.
  for(int i = 0; i < gSensorBufferSize*2; i++) {
    processLightSensor();
    delay(500);
  }
  gInitLightSensorYn = true;      
}

void processLightSensor() {
  // 1. 버퍼에 추가. FIFO처리
  int ligthValue = analogRead(LIGHT_SENSOR_PIN);
  pushArr(gLightValueArr, ligthValue);

  // 2. 평균값 계산
  if(gInitLightSensorYn) {
    gLightValue = getSum(gLightValueArr) / gSensorBufferSize;
  } else {
    return;
  }  
}

void initDhtSensor() {
  // Soil Sensor 초기화. 정상수치 나오는데, 5초 정도 걸림.
  for(int i = 0; i < gSensorBufferSize*2; i++) {
    processDhtSensor();
    delay(500);
  }
  gInitDhtSensorYn = true;    
}

// DHT 센서 처리
void processDhtSensor() {
  // DHT 센서 처리 ////////////////////////////
  // 1. 버퍼에 추가. FIFO처리
  pushArr(gDhtHumidityArr, dht.readHumidity());
  pushArr(gDhtTemperatureArr, dht.readTemperature());

  if(gInitDhtSensorYn) {
    // 2. 평균값 계산
    gDhtHumidity = getSum(gDhtHumidityArr) / gSensorBufferSize;
    gDhtTemperature = getSum(gDhtTemperatureArr) / gSensorBufferSize;    
  } else {
    return;
  }

  // Check if any reads failed and exit early (to try again).
  if (isnan(gDhtHumidity) || isnan(gDhtTemperature)) {
    sMessage += "\tFailed to read from DHT sensor!";
    return;
  }
}

void initSoilSensor() {
  // Soil Sensor 초기화. 정상수치 나오는데, 5초 정도 걸림.
  for(int i = 0; i < gSensorBufferSize*2; i++) {
    processSoilSensor();
    delay(500);
  }
  gInitSoilSensorYn = true;  
}

void processSoilSensor() {
  // 토양 센서 처리 //////////////////////////
  // TODO pin을 1, 2로 분리해야 함.
  int soilAnalogValue = analogRead(SOIL_SENSOR_PIN1);
  if (!soilRhMaxWhenAnalHighYn) {
    soilAnalogValue = 1023 - soilAnalogValue;
  }

  // Max(물에 잠길때) : 540 ~ 630, Min: 0
  // 2차 측정 : Max 760
  // 변환된 값 Max를 1023이라고 가정하고 digitalValue 계산
  int soilDigitalValue = map(soilAnalogValue, 0, 1023, 0, 100);
  soilDigitalValue = constrain(soilDigitalValue, 0, 100);

  // 1. 버퍼에 추가. FIFO처리
  pushArr(gSoilAnalogArr, soilAnalogValue);
  pushArr(gSoilDigitalArr, soilDigitalValue);
  
  if(gInitSoilSensorYn) {
    gSoilAnalogValue = getSum(gSoilAnalogArr) / gSensorBufferSize;
    gSoilDigitalValue = getSum(gSoilDigitalArr) / gSensorBufferSize;    
  } else {
    return;
  }

  if (WATER_RELAY_USE_YN && (gSoilDigitalValue < soilHumidityIndex)) {
    // 습도가 0보다는 크고, 인덱스값보다 작을 때 물을 공급한다.
    if (gSoilDigitalValue < 3) {
      sMessage += "\t토양센스 Not work"; // 3미만은 미측정으로 처리한다.
    } else {
      waterDelayTime -= loopIntervalTime;
      if (waterDelayTime < 1) { // 물공급 인터벌 시간이 지나야, 추가로 물을 공급할 수 있다.
        waterDelayTime = waterSupplyIntervalTime; // 물공급 지연 시간
        digitalWrite(SOIL_RELAY_PIN1, HIGH);
        sMessage += "\trelay on(";
        sMessage += String(waterSupplyTime);
        sMessage += "초)";
        delay(waterSupplyTime * 1000);     // 물주기 시간
        digitalWrite(SOIL_RELAY_PIN1, LOW);
        sMessage += "\trelay off";
      } else {
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
    tempMessage += "H: "; // Humidity
    tempMessage += String(gDhtHumidity);
    tempMessage += "%\t";

    tempMessage += "T: "; // Temperature
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
    tempMessage += "\t";
  }

  if(LIGHT_SENSOR_USE_YN) {
    tempMessage += "Light(Raw): ";
    tempMessage += String(gLightValue);    
  }

  tempMessage += sMessage;
  if (tempMessage.length() < 1) {
    tempMessage = "No Message";
  }

  tempMessage.replace("\t",",\t");
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
    u8g2.drawStr(2, 12, ("v"+gVersion+",IP: " + gIp).c_str()); // version and ip
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


void pushArr(float intArr[], float tempValue) {
  for(int i = 1 ; i < gSensorBufferSize; i++) {
    intArr[i-1] = intArr[i];
  }
  intArr[gSensorBufferSize-1] = tempValue;
}

float getSum(float intArr[]) {
    float tempSum = 0;
    for(int i = 0 ; i < gSensorBufferSize; i++) {
      tempSum += intArr[i];
    }  
    return tempSum;
}

