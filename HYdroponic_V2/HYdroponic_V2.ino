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
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40
#define numData 9
#define dataPerScreen lcdHeight //I don't think this is proper form.

//BUTTONS connected to digital pins X
#define back 4
#define enter 6
#define next 9
#define last 5

int lcdWidth = 16;
int lcdHeight = 2;
LiquidCrystal_I2C lcd(0x27, lcdWidth, lcdHeight);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
DHT dht(DHTPIN, DHTTYPE);
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);
int millisUpdate = millis();
bool nextState = HIGH;
bool lastState = HIGH;
bool enterState = HIGH;
bool backState = HIGH;
bool pressedOnce = HIGH;
unsigned long buttonOffset = 0;
int buttonDelay = 300;
int arrowPosition = 0;
bool calibrateMenu = false;        
float Offset= 0;     //deviation compensate of ph sensor

int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;  

//variables for all the data
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
unsigned long sendDataInterval = 10000;//for 1 hour 3600000
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
  calibratePhSensor();
}

void loop() {
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
    if(!calibrateMenu && (arrowPosition + dataPerScreen * screenCount == 3 )|| calibrateMenu){
    if(!calibrateMenu){
    calibrateMenu = true;
    }
    else if(calibrateMenu && arrowPosition == 0){
      calibrateMenu = false;//DON'T DO ANYTHING
    }
    else if(calibrateMenu && arrowPosition >= 1){
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
  return((String)(analogRead(A6) > ambLS));//400 being ambient light
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
  if (!calibrateMenu){
  lcd.setCursor(0,arrowPosition);
  lcd.print("=>");
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
    lcd.print("=>");
    lcd.setCursor(2, 1);
    lcd.print("No");
    lcd.setCursor(8,1);                                                               
    lcd.print("Yes");
  }
}
void calibratePhSensor(){
        Offset = 0;
        gatherData();
        Offset = 7.00 - data[3].toFloat();
        gatherData();
}

