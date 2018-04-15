#define ambLS 400
#define numLights 4

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
int lcdWidth = 16;
int lcdHeigth = 2;
LiquidCrystal_I2C lcd(0x27, lcdWidth, lcdHeigth);
int millisUpdate = millis();
/*
float waterTemp = 22.3;//Change this variables or make your or make your own ones
float airHumidity = 12.3;
float airTemp = 45.2;
float ph= 7.00;
float waterlevel = 10.3;
float light = 189;
*/

// ---------------------------------------------------------------------------
// Example NewPing library sketch that does a ping about 20 times per second.
// ---------------------------------------------------------------------------

#include <NewPing.h>

#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 350 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
//BUTTONS
#define back 4
#define enter 6
#define next 9
#define last 5
bool nextState = HIGH;
bool lastState = HIGH;
bool enterState = HIGH;
bool backState = HIGH;
bool pressedOnce = HIGH;
unsigned long buttonOffset = 0;
int buttonDelay = 300;
int arrowPosition = 0;
bool calibrateMenu = false;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

#include "DHT.h"

#define DHTPIN 8     // what digital pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);


/*********
  Rui Santos
  Complete project details at http://randomnerdtutorials.com  
  Based on the Dallas Temperature Library example
*********/

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is conntec to the Arduino digital pin 2
#define ONE_WIRE_BUS 7

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);


/*
 # This sample code is used to test the pH meter V1.0.
 # Editor : YouYou
 # Ver    : 1.0
 # Product: analog pH meter
 # SKU    : SEN0161
*/

#define SensorPin A0            //pH meter Analog output to Arduino Analog Input 0
float Offset= 0;            //deviation compensate
//#define LED 4
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;  



//variables for all the data
#define numData 9
#define dataPerScreen lcdHeigth
uint8_t screenCount = 0;
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


unsigned long sendDataInterval = 10000;
//unsigned long screenChangeInterval = 3000;
unsigned long lastSendDataMillis;
//unsigned long lastScreenChangeMillis;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(enter,INPUT_PULLUP);
  pinMode(next, INPUT_PULLUP);
  pinMode(last, INPUT_PULLUP);
  pinMode(back, INPUT_PULLUP);
  dht.begin();
  sensors.begin();
  
  lcd.begin();
  
//  lastScreenChangeMillis = millis();
//  lastSendDataMillis = lastScreenChangeMillis;
  gatherData();
}

void loop() {
  // put your main code here, to run repeatedly:
  //every 2s, send new values.

  if((millis() - lastSendDataMillis) > sendDataInterval){
    gatherData();
  }
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
    if(!calibrateMenu && arrowPosition + dataPerScreen * screenCount == 3|| calibrateMenu){
    if(!calibrateMenu){
    calibrateMenu = true;
    }
    else if(calibrateMenu && arrowPosition == 0){
      calibrateMenu = false;//GO BACK
    }
    else if(calibrateMenu && arrowPosition >= 1){
      if(arrowPosition + dataPerScreen * screenCount == 3){
        Offset = 0;
        gatherData();
        Offset = 7.00 - data[3].toFloat();
      }
      calibrateMenu = false;//DO SOMETHING
    }
    arrowPosition = 0;
    gatherData();
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
  return((String)(analogRead(A1) > ambLS));//400 being ambient light
}
String getLight2Status(){
  if(numLights == 1) return "X";
  return((String)(analogRead(A2) > ambLS));//400 being ambient light
}
String getLight3Status(){
  if(numLights == 1) return "X";
  return((String)(analogRead(A3) > ambLS));//400 being ambient light
}
String getLight4Status(){
  if(numLights == 1) return "X";
  return((String)(analogRead(A4) > ambLS));//400 being ambient light
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

void showLCDData(){
  
  lcd.clear();
  /*
  delay(200);
  lcd.setCursor(0,0);
  lcd.print(screenCount);
  */
  if (!calibrateMenu){
  lcd.setCursor(0,arrowPosition);
  lcd.print(">");
  for(int i = 0; i < dataPerScreen; i++){
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
    lcd.print(">");
    lcd.setCursor(2, 1);
    lcd.print("No");
    lcd.setCursor(8,1);                                                               
    lcd.print("Yes");
  }
}
