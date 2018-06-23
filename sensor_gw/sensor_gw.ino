#include <AES.h>
#include "crc16.h"
#include "protocol.h"
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>

#define FIREBASE_HOST "ppd-light.firebaseio.com"
#define FIREBASE_AUTH "zC4cl98ldKnMieP0MX3M6Mao9Eqc4oixk8Hh2xW1"
#define WIFI_SSID "Sengsly"
#define WIFI_PASSWORD "thaikims3ng"

char  serialData[MAX_TX_SIZE];
SoftwareSerial softSerial(5, 4); // RX, TX

//Firebase path
String eventType;
String eventPath;
String eventData;




unsigned int voltage_l1=0;          //V
unsigned int voltage_l2=0;          //V
unsigned int voltage_l3=0;          //V
unsigned int current_l1=0;          //A
unsigned int current_l2=0;          //A
unsigned int current_l3=0;          //A
unsigned int register_id=0;         //register id

//config_struct param;

//byte configEEPROM[EE_MAXNUM];       //an array in SRAM

data_struct send_data;

//AES Encryption================================================
byte iv [N_BLOCK] ;
byte data[MAX_TX_SIZE];
byte cipher[MAX_TX_SIZE];
byte check [MAX_TX_SIZE] ;
unsigned long long int my_iv = 0x230D09A;
byte key[] = "ppd01thaikimseng";
AES aes;
//AES Encryption================================================

//raw_struct rt;

void setup() {

  // put your setup code here, to run once:
  Serial.begin(9600);

  //Setup soft serial
  softSerial.begin(9600);
  

  //Setup Wifi and firebase==============================

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.stream("DP/");  

  //======================================================
}
bool encrypt(data_struct sendData){
  aes.iv_inc();
  aes.set_IV(my_iv);
  aes.get_IV(iv);
  memcpy(data,&sendData,sizeof(sendData));
  aes.do_aes_encrypt(data,sizeof(sendData)+1,cipher,key,128,iv);
  return true;
}

bool processMessage(data_struct data){
  raw_struct para;
  data_struct localData;
  
  switch(data.commandType){
    case command_enum::realtimeData:
      //memcpy(&para,  &data.data[0],sizeof(raw_struct));
      sendToServer(&data);

      break;
    case command_enum::setPara:
      //Set parameter for the configuration
      //memcpy(&para,  (void*)data->data[0],sizeof(raw_struct));
      //memcpy//
      //memcpy(&para,&para,sizeof(raw_struct));
      //sendToGateway (command_enum::responseSetPara, &localData);
      break;  
    case command_enum::getPara:
//      para.long1=param.timer_off_time;      
//      para.long2=param.timer_on_time;      
//      memcpy(&localData.data[0], &para ,sizeof(raw_struct));
//      sendToGateway (command_enum::responseGetPara, &localData);
      
      break;
    case command_enum::getTime:
      //para.long1=rtc.getUnix;     
//      para.long1=(long)now.unixtime();      
//      memcpy(&localData.data[0], &para ,sizeof(raw_struct));
//      sendToGateway (command_enum::responseGetTime, &localData);
      break;
    case command_enum::setTime:
//      memcpy(&para,localData.data[0],sizeof(raw_struct));
//      setUnixTime (para.long1);
//      para.long1=1;     //success
      //memcpy(&localData.data[0], &para ,sizeof(raw_struct));
 //     sendToGateway (command_enum::responseSetTime, &localData);
      // (void*)data
      break;
  }
}
void setUnixTime(long){

}
void SynchronizeTimer(){
  
}
bool sendToServer(data_struct *data){
  String path=String("NE/"+ String(data->id) +  "/Realtime");
  raw_struct rt;
  DynamicJsonBuffer jBuffer;
  JsonObject& realtime= jBuffer.createObject();
  memcpy(&rt,  &data->data[0],sizeof(raw_struct));

  realtime["L1"]  = rt.long1;
  realtime["L2"]  = rt.long2;
  realtime["L3"]  = rt.long3;
  realtime["I1"]  = rt.word1;
  realtime["I2"]  = rt.word2;
  realtime["I3"]  = rt.word3;
  realtime["Time"]= data->timestamp;

  Firebase.set(path, realtime);
  Firebase.stream("DP/");  

  return true;
}
bool sendToElement(command_enum msg, int Address, int Channel,raw_struct raw ){
  byte buf[MAX_TX_SIZE+3];      //Address and channel
  data_struct data;
  // Point to point E32 
  //0011 0A 1111 1112 1111 11109 0008 0144444546474849
  //00 11 Modules address
  //0A Modules channel  
  //Remaining Data

  //#define SET_RADIO_CH    0x17
  //#define SET_RADIO_AD    0x18
  //#define SET_RADIO_GW    0x19

  buf[0]=(Address & 0xff00) >> 8;
  buf[1]=(Address & 0xff);
  buf[2]=Channel;
  data.id=0;
  data.crc=CRC16;
  data.register_id=0x10;
  data.commandType=msg;
  
  memcpy ( &data.data[0], &raw ,sizeof(data_struct));
  memcpy(&buf[3], &data ,sizeof(data_struct));
  softSerial.write((uint8_t *)&data,8);
  softSerial.write(buf,MAX_TX_SIZE);
  Serial.write(buf,MAX_TX_SIZE);
  return true;
}
void loop() {
  if (Firebase.available()) {
    FirebaseObject event = Firebase.readEvent();
    eventType = event.getString("type");
    eventType.toLowerCase();
     
    if (eventType == "put") {


      Serial.print("event: ");
      Serial.println(eventType);

      eventData=event.getString("data");
      eventPath=event.getString("path");
      // /1/Time

      //Radio_AD: 
      //Radio_CH: 

      Serial.print("data: ");
      Serial.println(eventData);
      Serial.print("path: ");
      Serial.println(eventPath);
      int charIndex=eventPath.lastIndexOf("/");
      if( charIndex>=1){

        eventPath=eventPath.substring(0,charIndex);
        Serial.print("String found =");
        Serial.println(eventPath );
        raw_struct raw;
        raw.long1=200;
        raw.long2=200;
        raw.long2=200;
        raw.word1=200;
        raw.word2=200;
        raw.word3=200;
        int NE_Radio=Firebase.getInt("DP/"+eventPath+"/Radio_AD");
        int NE_Channel=Firebase.getInt("DP/"+eventPath+"/Radio_CH");
        sendToElement(command_enum::setTime ,NE_Radio,NE_Channel,raw);
        Serial.print("Radio :"); Serial.println(NE_Radio);
        Serial.print("Channel :"); Serial.println(NE_Channel);        
      }
    }
  }   

  if (softSerial.available() ) {
    int cnt = 0;
    do {
      serialData[cnt] = softSerial.read();
      Serial.print(serialData[cnt],HEX);
      cnt++;
      delay(1);
    } while (softSerial.available() && cnt < MAX_TX_SIZE);
    Serial.println("");
    memcpy(&send_data,serialData,sizeof(send_data));
    if(send_data.crc==CRC16){//Check whether the CRC check iscorrect
      processMessage(send_data);
    };
    
  };

}