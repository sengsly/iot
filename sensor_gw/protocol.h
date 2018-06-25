#define GET_RTC_TIME        0x10
#define SET_RTC_TIME        0x11
#define SET_TMR_ON          0x12
#define SET_TMR_OFF         0x13
#define SET_NE_ID           0x16
#define SET_RADIO_CH        0x17
#define SET_RADIO_AD        0x18
#define SET_RADIO_GW_AD     0x19


#define EE_VOL_LIM  0x00
#define EE_CUR_LIM  0x02
#define EE_TIMER_ON 0x04
#define EE_TIMER_OFF    0x06
#define EE_CRC_CHECK    0x08
#define EE_MAXNUM       10
#define MAX_TX_SIZE      50
#define TIME_OUT         5000           //time out in ms
#define NE_ID         1001            //NE  id
#define RADIO_CH         0x0A             //Frequency channel
#define RADIO_AD        0x0011             //NE address
#define RADIO_GW_AD       0x1              //address

enum command_enum 
{   
    realtimeData = 1, 
    responseData = 2,
    requestData = 3,
    getPara=4,
    setPara =5,
    getTime=6,
    setTime=7,
    responseSetPara=8,
    responseGetPara=9,
    responseSetTime=10,
    responseGetTime=11

};


struct config_struct{
    word voltage_lim;
    word current_lim;
    word timer_on_time;
    word timer_off_time;
    word crcValue;
};
/*
V1		= 1 Bytes
V2		= 1 Bytes
V3		= 1 Bytes
I1		= 1 Bytes
I2		= 1 Bytes
I3		= 1 Bytes
*/
/*
unsigned int voltage_l1=0;          //V
unsigned int voltage_l2=0;          //V
unsigned int voltage_l3=0;          //V
unsigned int current_l1=0;          //A
unsigned int current_l2=0;          //A
unsigned int current_l3=0;          //A
*/

/*
struct realtime_struct{
    word L1;
    word L2;
    word L3;
    byte I1;
    byte I2;
    byte I3;
};
*/
/*
struct realtime_struct{
    word L1;
    word L2;
    word L3;
    byte I1;
    byte I2;
    byte I3;
};
*/
struct raw_struct{
    unsigned long long1;
    unsigned long long2;
    unsigned long long3;
    word word1;
    word word2;
    word word3;
};
struct param_struct{
    word timer_on_time;
    word timer_off_time;
    word rtc_time;
};

struct response_struct{
    word register_id;
    word data;
};

struct request_struct{
    word register_id;
    word data;
};
struct data_struct{
    word id;
    word crc;
    word register_id;
    word commandType;
    long timestamp;
    byte data[18];
};

