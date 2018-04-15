//https://www.bc-robotics.com/tutorials/using-a-flow-sensor-with-arduino/
//pullup resistor (10K) on signal wire


#define flowRatePerRotation 2.25
int flowPin = 2;    //This is the input pin on the Arduino
double flowRate;    //This is the value we intend to calculate.
double flowPercent; //0 to 10 value 
volatile int count; //This integer needs to be set as volatile to ensure it updates correctly during the interrupt process.  

int optimalFlowRate = 1; // L per minute

void setup() {
  // put your setup code here, to run once:
  pinMode(flowPin, INPUT);           //Sets the pin as an input
  attachInterrupt(0, Flow, RISING);  //Configures interrupt 0 (pin 2 on the Arduino Uno) to run the function "Flow"  
  Serial.begin(9600);  //Start Serial
}
void loop() {
  // put your main code here, to run repeatedly:  
  
  Serial.println(getCurrentFlowRate());
  delay(1000);
}

void Flow()
{
   count++; //Every time this function is called, increment "count" by 1
}

void getCurrentFlowRate(){
  count = 0;      // Reset the counter so we start counting from 0 again
  interrupts();   //Enables interrupts on the Arduino
  delay (1000);   //Wait 1 second 
  noInterrupts(); //Disable the interrupts on the Arduino
  
  //Start the math
  flowRate = (count * flowRatePerRotation);        //Take counted pulses in the last second and multiply by 2.25mL 
  flowRate = flowRate * 60 / 1000;         //Convert seconds to minutes, mL to L, giving you L / Minute
  
  flowPercent = (flowRate / optimalFlowRate) * 100;
	return (flowPercent);
}

void calibrateFlowRate(){
	optimalFlowRate = getCurrentFlowRate;
}
