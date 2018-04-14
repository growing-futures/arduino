
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_Address 0x3C
Adafruit_SSD1306 oled(1);
 
void setup() {
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_Address);

}
 
void loop() {
  float waterTemp = 22.3;//Change this variables or make your or make your own ones
  float airHumidity = 12.3;
  float airTemp = 45.2;
  float ph= 7.00;
  float waterlevel = 10.3;
  float light = 189;
  oled.clearDisplay();
  oled.setTextColor(WHITE);
  oled.setCursor(0,0);
  oled.print("Wt: ");
  oled.print(waterTemp);
  oled.print("  Ah: ");
  oled.println(airHumidity);
  oled.print("At: ");
  oled.print(airTemp);
  oled.print("  Ls: ");
  oled.println(light);
  oled.print("Wl: ");
  oled.print(waterlevel);
  oled.print("  pH: ");
  oled.println(ph);
  
  oled.display();
}
