#define back 5
#define enter 4
#define next 3
#define last 2
bool nextState = HIGH;
bool lastState = HIGH;
bool enterState = HIGH;
bool backState = HIGH;
bool pressedOnce = HIGH;
unsigned long buttonOffset = 0;
int buttonDelay = 300;
void setup() {
  pinMode(enter,INPUT_PULLUP);
  pinMode(next, INPUT_PULLUP);
  pinMode(last, INPUT_PULLUP);
  pinMode(back, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
  enterState = digitalRead(enter);
  nextState = digitalRead(next);
  lastState = digitalRead(last);
  backState = digitalRead(back);
  if (millis() - buttonOffset > buttonDelay){
   if((nextState == LOW) && (!pressedOnce)){
    Serial.println("Next State");
    pressedOnce = true;
    buttonOffset = millis();
  }
  else if((lastState == LOW) && (!pressedOnce)){
    Serial.println("Last State");
    pressedOnce = true;
    buttonOffset = millis();
  }
  else if((enterState == LOW) && (!pressedOnce)){
    Serial.println("Enter State");
    pressedOnce = true;
    buttonOffset = millis();
  }
  else if((backState == LOW) && (!pressedOnce)){
    Serial.println("Back State");
    pressedOnce = true;
    buttonOffset = millis();
  }
  if ((enterState == HIGH) && (nextState == HIGH) && (lastState == HIGH)){
    pressedOnce = false;
  }
  }
  
}
