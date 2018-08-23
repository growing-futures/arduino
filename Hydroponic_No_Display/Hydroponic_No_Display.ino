#include <Wire.h> //for the LCD and the i2c comm rpi
#include "Config.h"

#define PH //using a ph sensor?
#define WL //using a water level sensor?
#define WF //using a water flow sensor?
#define LED_NOTIFICATIONS
#define I2C_COMM

unsigned long lastDataMillis;
unsigned long lastTimePumpRunning;
unsigned long lastHourMillis;
double lastHourFlow = 0;

#ifdef I2C_COMM
  uint8_t flag_int_to_send_to_PI = 0;
  uint8_t flag_int_received_from_PI = 0;
  String data_to_send_PI = "TESTINGTESTINGTESTING";
  //String data_received_from_pi = "";
  bool sendingI2C = false;
#endif

#ifdef PH
  //#include <EEPROMex.h>//to save the pH Offset
  //#include <EEPROMVar.h>//above
  #include <Vcc.h> //analog read accuracy
  //Measure VCC Library
  #define VccCorrection (1.0/1.0)  // Measured Vcc by multimeter divided by reported Vcc
  Vcc vcc(VccCorrection);
  //EEPROM
  //#define maxAllowedWrites 100
  //#define memBase          350
  //#define pHOffsetEEPROMAddress 400//this is set in the setup
  #define samplingInterval 20 //pH sampling interval
  #define printInterval 800 //amount of time to sample pH in milliseconds
  #define ArrayLength  10 //for the pH sensor readings - will allow to take an average reading
  int pHArray[ArrayLength]; //Store the average value of the sensor feedback
  uint8_t pHArrayIndex=0;
  //float pHOffset = 0.0;     //deviation compensate of ph sensor - should be stored in EEPROM
#endif

#ifdef WL
  #include <NewPing.h>//for ultrasonic water level sensor
  //ULTRASONIC SENSOR
  #define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
  // NewPing setup of pins and maximum distance.
  NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
#endif

#ifdef WF
  #define FLOW_RATE_FACTOR 11 //F=11Q   (Q = L/MIN) from web page.
  volatile int tickCount = 0;
  #define ONE_MINUTE_MILLIS 60000 //number of milliseconds in one minute
  unsigned long lastMinuteMillis = 0;
#endif

#define NUM_DATA 4
//variables for all the data, with their prefixes to display on the LCD
String dataPrefix[NUM_DATA];
float floatData[NUM_DATA];
String stringData[NUM_DATA];

#define NONE 4
#define PH_VALUE 1
#define WATER_FLOW_VALUE 2
#define WATER_LEVEL_VALUE 3
uint8_t problem = NONE;

void setup() {
  
   //Notification LEDs
  #ifdef LED_NOTIFICATIONS
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, HIGH);
    delay(1000);
  #endif

  Serial.begin(115200);
  //delay(3000);
  //Serial.println("Setup");
  //Serial.println("TEST");
  //Serial.flush();

  #ifdef PH
    //EEPROM.setMemPool(memBase, EEPROMSizeUno);
    //EEPROM.setMaxAllowedWrites(maxAllowedWrites);
      //pHOffsetEEPROMAddress = EEPROM.getAddress(sizeof(float)); //Using a defined address now
      //EEPROM.writeFloat(pHOffsetEEPROMAddress, 0.0);
    // pHOffset = EEPROM.readFloat(pHOffsetEEPROMAddress);
  #endif

  #ifdef WF
    pinMode(WATER_FLOW_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(WATER_FLOW_PIN), waterFlowISR, RISING);
    floatData[WATER_FLOW_INDEX] = 0.0;
  #endif
  
  //noInterrupts();

  #ifdef I2C_COMM
   Wire.begin(SLAVE_ADDRESS);
   Wire.onReceive(receiveData);
   Wire.onRequest(sendData);
   flag_int_to_send_to_PI = 1;
  #endif  

  lastDataMillis = millis();
  lastTimePumpRunning = 0;
  lastHourMillis = 0;
  gatherData();
    //while(1){
      //delay(9999);
      //gatherData();
  //}
}


void loop() {
  
  if(!sendingI2C){
    if(lastMinuteMillis > millis()){ lastMinuteMillis = millis();}
    if((millis() - lastMinuteMillis) > ONE_MINUTE_MILLIS){
        #ifdef WF
          checkFlow();
        #endif
        lastMinuteMillis = millis();
     }
     if(lastDataMillis > millis()){lastDataMillis = millis();}
     if((millis() - lastDataMillis) > DATA_RATE_MILLIS){
        gatherData();
        lastDataMillis = millis();
     }
  }
}


#ifdef WL
  double getWaterLevelCm(){
    double cm = sonar.ping()/2 * 0.0343;
    cm = constrain(cm, MAX_WATER_LEVEL, MIN_WATER_LEVEL);
    return(map(cm, MAX_WATER_LEVEL, MIN_WATER_LEVEL, 99, 0));
    //return(cm);
  } //using 343 m/s as speed of sound. Will be accurate enough for all our readings.
#endif

#ifdef PH
float getVINREF(){return(vcc.Read_Volts());}
float getPHValue(){return(getRawpHValue() + PH_OFFSET);}
float getRawpHValue(){
  //float pHValue,voltage;
    //pHValue = (analogRead(A0) * getVINREF() / 1024 * 3.5) + pHOffset;
    /*for(int i = 0; i < ArrayLength; i++){
      pHArray[i]=analogRead(SensorPin);
      voltage = avergearray(pHArray, ArrayLength)*getVINREF()/1024;
      pHValue = (3.5*voltage)+pHOffset;//Here is where we compensate for the offset
      delay(20);
    }*/
    //Serial.println(pHValue);
    //return(pHValue);
    return (((float)analogRead(A0)) * getVINREF() * 3.5 / 1024);
}
#endif

void gatherData(){

    String dataOutput = "";

    #ifdef WL
      floatData[WATER_LEVEL_INDEX] = (float)getWaterLevelCm();
      stringData[WATER_LEVEL_INDEX] = (String(floatData[WATER_LEVEL_INDEX],1)).substring(0,1);
      dataOutput += stringData[WATER_LEVEL_INDEX];
      dataOutput += ",";
    #endif
    #ifdef WF
      stringData[WATER_FLOW_INDEX] = String(floatData[WATER_FLOW_INDEX],1);
      dataOutput += stringData[WATER_FLOW_INDEX];
      dataOutput += ",";
    #endif
    #ifdef PH
      floatData[PH_INDEX] = (float)getPHValue();
      //floatData[RPH_INDEX] = (float)getRawpHValue();
      stringData[PH_INDEX] = String(floatData[PH_INDEX],2);
      //stringData[RPH_INDEX] = String(floatData[RPH_INDEX],2);
      dataOutput += stringData[PH_INDEX];
      dataOutput += ",";
      //dataOutput += stringData[RPH_INDEX];
      //dataOutput += ",";
    #endif

    dataOutput = dataOutput.substring(0,dataOutput.length()-1);

    Serial.println(dataOutput);
    #ifdef I2C_COMM
      data_to_send_PI = dataOutput;
    #endif

    attentionRequired();
    
    #ifdef LED_NOTIFICATIONS
      updateLEDNotifications();
    #endif
}

#ifdef LED_NOTIFICATIONS
void updateLEDNotifications(){//TODO - flow rate reset //TODO add notifications to LCD?
  if(problem != NONE){
        //turn warning light on
        digitalWrite(RED_LED_PIN, HIGH);
        digitalWrite(GREEN_LED_PIN, LOW);
      }
      else{ //all good
        digitalWrite(RED_LED_PIN, LOW);
        digitalWrite(GREEN_LED_PIN, HIGH);
      }
}
#endif

void attentionRequired(){
  if((floatData[PH_INDEX] < LOW_PH_THRESHOLD) || 
    (floatData[PH_INDEX] > HIGH_PH_THRESHOLD)){
      problem = PH_VALUE;
      return;
    }
  if(floatData[WATER_LEVEL_INDEX] < LOW_WATER_LEVEL_THRESHOLD){
    problem = WATER_LEVEL_VALUE;
    return;
  }
  if(floatData[WATER_FLOW_INDEX] < LOW_WATER_FLOW_THRESHOLD){
    problem = WATER_FLOW_VALUE;
    return;
  }
  problem = NONE;
}

#ifdef WF
void checkFlow(){ //run this every few minutes to check the flow rate
  tickCount = 0;//just for good measure
  interrupts();
  delay(2000);
  int tickCount2 = tickCount;
  //noInterrupts();
  double currentFlow = tickCount2 / FLOW_RATE_FACTOR/2;
  //lastTimePumpRunning
  //lastHourFlow
  //lastHourMillis
  //floatData[WATER_FLOW_INDEX]
  bool pumpRunning;
  if (currentFlow > (FLOW_RATE_EXPECTED/2.5)){
    pumpRunning = true;
    floatData[WATER_FLOW_INDEX] = currentFlow;
    lastTimePumpRunning = millis();
  }
  else{
    pumpRunning = false;
    if(lastTimePumpRunning > millis()){lastTimePumpRunning = millis();}
    if((millis() - lastTimePumpRunning) > (ONE_HOUR_MILLIS * 2)){
      //pump not running, and 2 hours since it last ran
      floatData[WATER_FLOW_INDEX] = currentFlow;
    }
    //if the pump is not running, but it is less than 2 hours since it last ran, do nothing
  }
  
  
  //return the maximum flow rate in the last hour

  //if less than an hour has passed since the last flow check
  //return max of last hour flow and current flow.
  //
  
  
  tickCount = 0;
}
void waterFlowISR(){
  tickCount++;
}
#endif

#ifdef PH
//This is for the pH sensor, don't touch it.
double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    //Serial.println(F("Error number for the array to avraging!/n"));
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}
#endif

#ifdef I2C_COMM
  void receiveData(int byteCount) {
  
      while(Wire.available()) {
          if(flag_int_received_from_PI == 100){
            Serial.print(F("New PH Calibration Value as ? : "));
            Serial.println(Wire.read());
          }
          else{
          flag_int_received_from_PI = Wire.read();
  
          if(flag_int_received_from_PI == 1) {
              Serial.println(F("PI Wants data[]."));
              flag_int_to_send_to_PI = 3;
              sendingI2C = true;
          }
  
          if(flag_int_received_from_PI == 3) {
             Serial.println(F("PI Wants To say its ok."));
             flag_int_to_send_to_PI = 1;
          }

          if (flag_int_received_from_PI == 4){
            Serial.println(F("Pi wants to calibrate PH"));
            sendingI2C = true;
            flag_int_to_send_to_PI = 4;
          }
          }
      }
  }
  void sendData() {
      if(flag_int_to_send_to_PI == 1) {
          Serial.println(F("CB 1 to PI"));
          Wire.write(flag_int_to_send_to_PI);
      }
  
      if(flag_int_to_send_to_PI == 3) {
        //The data is written here. data_to_send_PI is the string
        while(data_to_send_PI.length() < 32){
          data_to_send_PI += " ";
          //Serial.println(data_to_send_PI.length());
        }
        Wire.write(data_to_send_PI.c_str());
        sendingI2C = false; //leave this here, otherwise if the send fails, it gets stuck in the sendingI2C and does not restart the communication.  
      }
      if(flag_int_to_send_to_PI == 4){
        Serial.println(F("Ack"));
        Wire.write(flag_int_to_send_to_PI);
        flag_int_received_from_PI = 100;
      }

      
      flag_int_to_send_to_PI = 0;
  }
#endif

