#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);
int millisUpdate = millis();
float waterTemp = 22.3;//Change this variables or make your or make your own ones
float airHumidity = 12.3;
float airTemp = 45.2;
float ph= 7.00;
float waterlevel = 10.3;
float light = 189;
void setup()
{
  // initialize the LCD
  lcd.begin();

  lcd.home();
  lcd.print("Wt: ");
  lcd.print(waterTemp);
  lcd.setCursor(0,1);
  lcd.print("Ah: ");
  lcd.print(airHumidity);
}

void loop() {
  
  if (millis() - millisUpdate == 0){
  lcd.clear();
  lcd.home();
  lcd.print("Wt: ");
  lcd.print(waterTemp);
  lcd.setCursor(0,1);
  lcd.print("Ah: ");
  lcd.print(airHumidity);
  }
  else if(millis() - millisUpdate == 5000 ){
  lcd.clear();
  lcd.home();
  lcd.print("pH: ");
  lcd.print(ph);
  lcd.setCursor(0,1);
  lcd.print("At: ");
  lcd.print(airTemp);
  }
  else if(millis() - millisUpdate == 10000){
  lcd.clear();
  lcd.home();
  lcd.print("Ls: ");
  lcd.print(light);
  lcd.setCursor(0,1);
  lcd.print("Wl: ");
  lcd.print(waterlevel);
  }
  if (millis() - millisUpdate >= 15000){
    millisUpdate = millis();
  }
}
