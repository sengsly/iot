/*{
    "board": "arduino:avr:nano",
    "sketch": "sensor_ne\\sensor_ne.ino",
    "port": "COM9",
    "configuration": "cpu=atmega328"
}
*/

#include <Wire.h>
#include "RTClib.h"
#include "protocol.h"
#include "crc16.h"
#include "EEPROM.h"
#include <SoftwareSerial.h>
#include <AES.h>
#define RELAY_PIN 5
char  serialData[MAX_TX_SIZE];
DS1307 rtc;

SoftwareSerial softSerial(3, 4); // RX, TX


//AES Encryption================================================
byte iv [N_BLOCK] ;
byte data[MAX_TX_SIZE];
byte cipher[MAX_TX_SIZE];
//byte check [MAX_TX_SIZE] ;
unsigned long long int my_iv = 0x230D09A;
const   byte key[] PROGMEM= "ppd01thaikimseng";
byte buf[MAX_TX_SIZE+3];      //Address and channel

AES aes;
//AES Encryption================================================
//word L1;
//word L2;
//word L3;
//byte I1;
//byte I2;
//byte I3;
word oldTimeinminutes=0;
word voltage_l1=220;          //V
word voltage_l2=221;          //V
word voltage_l3=223;          //V
byte current_l1=5;          //A
byte current_l2=15;          //A
byte current_l3=7;          //A
unsigned int register_id=0;         //register id

config_struct param;
data_struct send_data;
raw_struct realtime;

byte configEEPROM[EE_MAXNUM];       //an array in SRAM

//char data[] = "Thai kimseng 012 907002"; //16 chars == 16 bytes




void setup() {

  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  //Setup soft serial
  softSerial.begin(9600);
  
  loadConfig();
  //param.timer_on_time=0x1100;
  //param.timer_off_time=0x0700;
  //saveConfig();

#ifdef AVR
  Wire.begin();
#else
  Wire1.begin(); // Shield I2C pins connect to alt I2C bus on Arduino Due
#endif
  rtc.begin();

    
  if (! rtc.isrunning()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }{
    Serial.println("Success reading RTC");
  }


  

  Serial.print("Time On : "); 
  Serial.println(param.timer_on_time);

  Serial.print("Time Off: "); 
  Serial.println(param.timer_off_time);
  //============Initialize realtime value
  fillRealtimeData();
  sendToGateway (command_enum::realtimeData, &send_data);
}


bool saveConfig(){
  
//  param.crcValue = gen_crc161(configEEPROM,EE_MAXNUM-3);
  memcpy(configEEPROM,&param,EE_MAXNUM);
  EEPROM.put(0,configEEPROM);
  Serial.print("Configuration saved, CRC : 0x");
  Serial.println(param.crcValue,HEX);
  return true;
}
bool loadConfig(){
  unsigned int crcvalue;

  EEPROM.get(0,configEEPROM);
  memcpy(&param,configEEPROM,EE_MAXNUM);
//  crcvalue = gen_crc161(configEEPROM,EE_MAXNUM-3);     //Remove CRC from the end
  Serial.print("EEPROM CRC = 0x");    
  Serial.println(param.crcValue, HEX);

  Serial.print("Calculate CRC = 0x");    
  Serial.println(crcvalue, HEX);

  if(param.crcValue!=crcvalue){
    Serial.print("Load default configuration\n");
    //corroupted crc value, load default configuration    
    param.voltage_lim=150;
    param.current_lim=10;
     param.timer_on_time=18*60+ 0;       //18:00 PM
    param.timer_off_time=8*60+ 0;       //08:00 AM
    return false;
  }else{
    Serial.print("Load configuratoin from EEPROM\n");
    return true;
  }
}


bool sendToGateway(command_enum msg,data_struct *data){
  // Point to point E32 
  //0011 0A 1111 1112 1111 11109 0008 0144444546474849
  //00 11 Modules address
  //0A Modules channel  
  //Remaining Data

  //#define SET_RADIO_CH    0x17
  //#define SET_RADIO_AD    0x18
  //#define SET_RADIO_GW    0x19

  buf[0]=(RADIO_AD & 0xff00) >> 8;
  buf[1]=(RADIO_AD & 0xff);
  buf[2]=RADIO_CH;
  data->id=NE_ID;
  data->crc=CRC16;
  data->register_id=0x10;
  data->commandType=msg;
  
  memcpy(&buf[3], (void*)data ,sizeof(data_struct));
  softSerial.write(buf,MAX_TX_SIZE);
  Serial.write(buf,MAX_TX_SIZE);
  return true;
}
void fillRealtimeData(){
  realtime.long1=voltage_l1;
  realtime.long2=voltage_l2;
  realtime.long3=voltage_l3;
  realtime.word1=current_l1;
  realtime.word2=current_l2;
  realtime.word3=current_l3;
  memcpy(&send_data.data[0],&realtime,sizeof(raw_struct));
}


bool processMessage(data_struct *data){
  raw_struct para;
  data_struct localData;
  DateTime now = rtc.now();
 
  switch(data->commandType){
    case command_enum::setPara:
      //Set parameter for the configuration
      memcpy(&para,localData.data[0],sizeof(raw_struct));
      param.timer_off_time=para.long1;
      param.timer_on_time=para.long2;
      para.long1=1;
      memcpy(&localData.data[0], &para ,sizeof(raw_struct));
      sendToGateway (command_enum::responseSetPara, &localData);
      break;  
    case command_enum::getPara:
      para.long1=param.timer_off_time;      
      para.long2=param.timer_on_time;      
      memcpy(&localData.data[0], &para ,sizeof(raw_struct));
      sendToGateway (command_enum::responseGetPara, &localData);
      break;
    case command_enum::getTime:
      
      para.long1=(unsigned long )now.unixtime();      
      memcpy(&localData.data[0], &para ,sizeof(raw_struct));
      sendToGateway (command_enum::responseGetTime, &localData);
      break;
    case command_enum::setTime:
      memcpy(&para,(void*)data.data[0],sizeof(raw_struct));
      Serial.print("Setting time=");
      Serial.println(para.long1);
      DateTime  newTime = DateTime(para.long1);
      rtc.adjust(newTime);
      para.long1=1;     //success
      memcpy(&localData.data[0], &para ,sizeof(raw_struct));
      sendToGateway (command_enum::responseSetTime, &localData);
      break;
  }
}
void loop() {
  DateTime now = rtc.now();
  word timeInMinutes=now.hour()*now.minute();
  if(oldTimeinminutes!=timeInMinutes){
    oldTimeinminutes=timeInMinutes;
    Serial.print("Time :" );
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    if ( (timeInMinutes>=param.timer_on_time) || (timeInMinutes <param.timer_off_time)){
      digitalWrite(RELAY_PIN,HIGH);
    }else{
      digitalWrite(RELAY_PIN,LOW);
    }

    if (( now.minute() % 2)==0){
      //Every 3 minutes send data to gateway    
      fillRealtimeData();
      send_data.timestamp=now.unixtime();
      sendToGateway (command_enum::realtimeData, &send_data);
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // turn the LED on (HIGH is the voltage level)
    }
  }  

  if (softSerial.available() ) {
    int cnt = 0;
    do {
      serialData[cnt++] = softSerial.read();
      delay(1);
    } while (softSerial.available() && cnt < MAX_TX_SIZE);
    
    Serial.println("");
    memcpy(&send_data,serialData,sizeof(send_data));
    if(send_data.crc==CRC16){   //Check if valid CRC data
      processMessage( &send_data);
    }
  }

}
