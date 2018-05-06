//Required Libraries
#include <OneWire.h> //for water temperature sensor
#include <DallasTemperature.h> //for water temperature sensor
#include <NewPing.h>//for ultrasonic water level sensor
#include <Wire.h> //for the LCD
#include <LiquidCrystal_I2C.h> //for the LCD
#include "DHT.h" //for air temperature and humidity sensor

#define DHTTYPE DHT11   // DHT 11 Air Temperature and humidity sensor - DHT22 are more accurate
#define DHTPIN 8  // what digital pin the DHT sensor is connected to.

#define SensorPin A0 //pH sensor signal pin

#define ambLS 400 // light sensors should return under this value when off, over when on
#define numLights 4 //number of lights. will send X otherwise

#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 350 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define ONE_WIRE_BUS 7 // data wire of the Water Temperature sensor

#define samplingInterval 20 //pH sampling interval
#define printInterval 800 //amount of time to sample pH in milliseconds
#define ArrayLenth  40 //for the pH sensor readings - will allow to take an average reading

#define numData 9 //total number of metrics to gather
#define dataPerScreen lcdHeight //how many metrics can fit on each screen (number of lines on LCD)

//BUTTONS connected to digital pins X
#define back 4
#define enter 6
#define next 9
#define last 5

//width and height of the LCD
#define lcdWidth 16
#define lcdHeight 2

//start the LCD library - i2c address, height, and width
LiquidCrystal_I2C lcd(0x27, lcdWidth, lcdHeight);

// NewPing setup of pins and maximum distance.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 

//start the DHT sensor with the pin and type
DHT dht(DHTPIN, DHTTYPE);

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

int millisUpdate = millis(); //I dont think this is used FIX

//states for all the buttons - inverted logic because of pullup resistors on the buttons for the menu
bool nextState = HIGH;
bool lastState = HIGH;
bool enterState = HIGH;
bool backState = HIGH;
bool pressedOnce = HIGH;

unsigned long buttonOffset = 0; //the timestamp of the last button press
#define buttonDelay 300 //minimum time (in milliseconds) between button presses
int arrowPosition = 0; //store the arrow position on the LCD
bool calibrateMenu = false; //
float Offset= 0;     //deviation compensate of ph sensor - should be stored in EEPROM

int pHArray[ArrayLenth]; //Store the average value of the sensor feedback
int pHArrayIndex=0;

uint8_t screenCount = 0;//what sensor display screen we are currently looking at

//variables for all the data, with their prefixes to display on the LCD
String dataPrefix[numData] = {"wT", "aH", "aT", "pH", "wL", "LS1", "LS2", "LS3", "LS4"};
String data[numData];
float wT;
float aH;
float aT;
float pH;
unsigned long wL;
String l1S;
String l2S;
String l3S;
String l4S;

//#define sendDataInterval 3600000 //for 1 h
#define sendDataInterval 10000//for 10 seconds
unsigned long lastSendDataMillis; //last time data was sent

//no longer using this, controlled by buttons. Maybe if no button pushed in X time, then auto-scroll
//unsigned long screenChangeInterval = 3000; 
//unsigned long lastScreenChangeMillis;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //setup for button pins
  pinMode(enter,INPUT_PULLUP);
  pinMode(next, INPUT_PULLUP);
  pinMode(last, INPUT_PULLUP);
  pinMode(back, INPUT_PULLUP);
  
  //setup for sensors
  dht.begin();
  sensors.begin();
  
  //setup for LCD
  lcd.begin();
  
  //this should not be here! should load offset from EEPROM - FIX
  calibratePhSensor();
}

void loop() {
  
  //if the set amount of time has passed since we last gathered, gather and send new data
  if((millis() - lastSendDataMillis) > sendDataInterval){ 
    gatherData();
  }
  
  //read all the buttons, if any are pressed, then take appropriate action.
  enterState = digitalRead(enter);
  nextState = digitalRead(next);
  lastState = digitalRead(last);
  backState = digitalRead(back);
  if (millis() - buttonOffset > buttonDelay){
   if((nextState == LOW) && (!pressedOnce)){
    arrowPosition++;
    if (arrowPosition >= dataPerScreen){
    arrowPosition = 0;
    screenCount++;
    }
    if (screenCount > numData/dataPerScreen){
      screenCount = 0;
    }
    pressedOnce = true;
    buttonOffset = millis();
    showLCDData();
  }
  else if((lastState == LOW) && (!pressedOnce)){
    arrowPosition--;
    if(arrowPosition <= -1){
    arrowPosition = 1;
    screenCount--;
    }
    if(screenCount== 255){
      screenCount =numData/dataPerScreen;
    }
    Serial.print(screenCount);
    pressedOnce = true;
    buttonOffset = millis();
    showLCDData();
  }
  else if((enterState == LOW) && (!pressedOnce)){
    if(!calibrateMenu && (arrowPosition + dataPerScreen * screenCount == 3 )|| calibrateMenu){
    if(!calibrateMenu){
    calibrateMenu = true;
    }
    else if(calibrateMenu && (arrowPosition == 0)){ //arrow was on NO
      calibrateMenu = false;//DON'T DO ANYTHING
    }
    else if(calibrateMenu && (arrowPosition >= 1)){ //arrow was on YES
      if(arrowPosition + dataPerScreen * screenCount == 3){
        calibratePhSensor();//CALIBRATE PH SENSOR
      }
      if(arrowPosition + dataPerScreen * screenCount == 9){
        //CALIBRATE FLOW SENSOR
      }
      calibrateMenu = false;
    }
    arrowPosition = 0;
    showLCDData();
    }
    pressedOnce = true;
    buttonOffset = millis();
  }
  else if((backState == LOW) && (!pressedOnce)){
    gatherData();
    pressedOnce = true;
    buttonOffset = millis();
  }
  if ((enterState == HIGH) && (nextState == HIGH) && (lastState == HIGH)&& (backState == HIGH)){
    pressedOnce = false;
  }
  }
}

unsigned long getWaterLevelCm(){
  return(sonar.ping_cm());
}

float getAirhumid(){
  return(dht.readHumidity());
}

float getAirTempC(){
  return(dht.readTemperature());
}

float getWaterTempC(){
  sensors.requestTemperatures();
  return(sensors.getTempCByIndex(0));
}

String getLight1Status(){
  return((String)(analogRead(A1) > ambLS));
}
String getLight2Status(){
  if(numLights == 1) return "X";
  return((String)(analogRead(A2) > ambLS));
}
String getLight3Status(){
  if(numLights == 1) return "X";
  return((String)(analogRead(A3) > ambLS));
}
String getLight4Status(){
  if(numLights == 1) return "X";
  return((String)(analogRead(A6) > ambLS));
}

float getPHValue(){
  float pHValue,voltage;
    for(int i = 0; i < 40; i++){
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
      pHValue = (3.5*voltage)+Offset;
      delay(20);
    }
    return(pHValue);
}

//**************************IF YOU WANT TO ADD NEW SENSORS, create a getXXXX function here, like outlined below.
//also, add this function to gatherData below, and put the new data in the arrays at the top, and increment numData
//This will also print "LOAD" on the LCD as it is gathering data.
/*
float getSensorData(){
  //get the data you need - read a pin, call a library, or whatever
  return(data)
}
*/


void gatherData(){
  lcd.setCursor(12,0);
    lcd.print("Load");
    wT = getWaterTempC();//Change this variables or make your or make your own ones
    aH = getAirhumid();
    aT = getAirTempC();
    pH= getPHValue();
    wL = getWaterLevelCm();
    l1S = getLight1Status();
    l2S = getLight2Status();
    l3S = getLight3Status();
    l4S = getLight4Status();

    data[0] = (String)wT;
    data[1] = (String)aH;
    data[2] = (String)aT;
    data[3] = (String)pH;
    data[4] = (String)wL;
    data[5] = (String)l1S;
    data[6] = (String)l2S;
    data[7] = (String)l3S;
    data[8] = (String)l4S;

    String dataOutput = "";
    dataOutput += wL;
    dataOutput += ",";
    dataOutput += aH;
    dataOutput += ",";
    dataOutput += aT;
    dataOutput += ",";
    dataOutput += wT;
    dataOutput += ",";
    dataOutput += pH;
    dataOutput += ",";
    dataOutput += l1S;
    dataOutput += ",";
    dataOutput += l2S;
    dataOutput += ",";
    dataOutput += l3S;
    dataOutput += ",";
    dataOutput += l4S;

    Serial.println(dataOutput);

    lastSendDataMillis = millis();
    lcd.setCursor(12,0);
    lcd.print("    ");
    showLCDData();
}

//This is for the pH sensor, don't touch it.
double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
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

//This shows all the data on the LCD, using the screenCount global variable to decide which set to show.
void showLCDData(){
  lcd.clear();
  if (!calibrateMenu){
  lcd.setCursor(0,arrowPosition);
  lcd.print("=>");
  for(int i = 0; i < dataPerScreen; i ++){
    if((i+ (dataPerScreen * screenCount)) <= numData-1){
      lcd.setCursor(2,i);
      lcd.print(dataPrefix[i + (dataPerScreen * screenCount)]);
      lcd.print(": ");
      lcd.print(data[i + (dataPerScreen * screenCount)]);
    }
  }
  }
  else{
    lcd.setCursor(0,0);
    lcd.print("Calibrate?");
    lcd.setCursor(6 * arrowPosition,1);
    lcd.print("=>");
    lcd.setCursor(2, 1);
    lcd.print("No");
    lcd.setCursor(8,1);                                                               
    lcd.print("Yes");
  }
}

//Calibrates the Offset on the pH sensor.
void calibratePhSensor(){
        Offset = 0;
        gatherData();
        Offset = 7.00 - data[3].toFloat();
        gatherData(); //Do not need to regather at the end necessarily.
}
