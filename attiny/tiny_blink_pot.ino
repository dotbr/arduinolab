const int led1 = 0; //pin 5
const int led2 = 1; //pin 6
const int pot = A1; //pin 7

unsigned long previousMillis = 0;

int ledState = LOW;

void setup() {
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

  //Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  int potVal = analogRead(pot);
  int potDelay = map(potVal, 0, 1023, 50, 1000);

  //Serial.println(potDelay);
  
  unsigned long currentMillis = millis();

    if(currentMillis - previousMillis >= potDelay) {
      // a última vez q fez o switch do led
      previousMillis = currentMillis;   
  
      // inverte o led:
      if (ledState == LOW)
        ledState = HIGH;
      else
        ledState = LOW;
  
      digitalWrite(led1, ledState);
      digitalWrite(led2, !ledState);
    }

}
