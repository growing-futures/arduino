#ifndef Config.h
#define Config.h

//*******************PIN Connections - I2C to the Pi as well
#define GREEN_LED_PIN 11 
#define RED_LED_PIN 4
#define SensorPin A0 //pH sensor signal pin
#define TRIGGER_PIN 10
#define ECHO_PIN 10
#define WATER_FLOW_PIN 2

//**************************ORDER and Prefix
#define PH_INDEX 0
#define WATER_LEVEL_INDEX 1
#define WATER_FLOW_INDEX 2
#define RPH_INDEX 3

#define PH_PREFIX "pH"
#define WATER_LEVEL_PREFIX "wL"
#define WATER_FLOW_PREFIX "wF"
#define RPH_PREFIX "rpH" //raw pH value measure by the sensor

//*********************Threshold Warning Values
#define LOW_PH_THRESHOLD 5.0
#define HIGH_PH_THRESHOLD 7.0
#define LOW_WATER_FLOW_THRESHOLD 5.0 // L/min - from the ProjectSucseed pump, we were getting around 16 L/min
#define LOW_WATER_LEVEL_THRESHOLD 20 //20% water level

#define MAX_WATER_LEVEL 8 //distance cm from sensor to water when reservoir is full
#define MIN_WATER_LEVEL 22 //distance cm from sensor to water when reservoir is empty

//This value is not used, but might be useful later.
#define FLOW_RATE_EXPECTED 15  //in L/M     370GPH is 23L/M

//********************I2C Communication address
#define SLAVE_ADDRESS 0x12

#define DATA_RATE_MILLIS (1*15*1000) //one minute for now. (15 secs)
#define ONE_HOUR_MILLIS (60*60*1000) //number of millis in one hour

//******************Calibration
#define PH_OFFSET 0.0

#endif
