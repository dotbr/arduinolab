#include <SoftwareSerial.h>
#include <Servo.h>

//Bluetooth
const int rxpin = 2;
const int txpin = 3;
SoftwareSerial bluetooth(rxpin, txpin);
String inputCommand = "";
bool inputComplete = false;

//Servo
const int servo = 11;
Servo motor;
const int pot = A2; //esse é o pino onde está o potenciômetro
int potVal;
int potAngulo;
int voiceAngulo = 90;
bool potLigado = false;

//sensor luz
const int light = A0;
int lightValue;
int lightLow = 1023;
int lightHigh = 0;
int lightMiddle;
bool falanteLigado = false;
const int falante = 6;

const int led = 9;
int ledState = LOW;
long blinkInterval = 1000;
unsigned long previousMillis = 0;
bool blinkLed = false;


const int lampada = 12; //pino do relê
bool lampadaSensor = false;
bool lampadaLigada = false;

void setup() {
  
  Serial.begin(9600);
  bluetooth.begin(9600);

  motor.attach(servo);
  
  Serial.println("Serial OK");
  
  pinMode(led,OUTPUT);
  pinMode(lampada,OUTPUT);

  digitalWrite(led,HIGH); //o led fica aceso na inicialização para fazer a calibragem do sensor de luz.
  //enquanto o led estiver aceso no boot, cubra e descubra o sensor com o dedo.
  while (millis() < 5000) {
    // record the maximum sensor value
    lightValue = analogRead(light);
    if (lightValue > lightHigh) {
      lightHigh = lightValue;
    }
    // record the minimum sensor value
    if (lightValue < lightLow) {
      lightLow = lightValue;
    }
  }
  
  lightMiddle = (lightHigh - lightLow) / 2;
  
  digitalWrite(led,LOW);
}

void loop() {

  unsigned long currentMillis = millis();
  
  potVal = analogRead(pot);
  potAngulo = map(potVal, 0, 1023, 0, 179);
  lightValue = analogRead(light);
  
  if (bluetooth.available()){
    char c = (char)bluetooth.read();
    inputCommand += c;
    if (c == '#')
    {
      inputComplete = true;
      Serial.println(inputCommand);
    }
  }
  
  if (inputComplete)
  {
    if (inputCommand == "*desligar led#")
    {
      digitalWrite(led, LOW);
      blinkLed = false;
    }
    
    if (inputCommand == "*ligar led#")
    {
      digitalWrite(led, HIGH);
      blinkLed = false;
    }
    
    if (inputCommand == "*piscar led#")
      blinkLed = true;

    if (inputCommand == "*rápido#")
      blinkInterval = blinkInterval / 2;

    if (inputCommand == "*devagar#")
      blinkInterval = blinkInterval * 2;
    
    if (inputCommand == "*ligar giro#")
      potLigado = true;
      
    if (inputCommand == "*desligar giro#")
      potLigado = false;

    if (inputCommand == "*direita#")
      voiceAngulo += 30;

    if (inputCommand == "*esquerda#")
      voiceAngulo -= 30;

    if (inputCommand == "*tudo direita#")
      voiceAngulo = 179;

    if (inputCommand == "*tudo esquerda#")
      voiceAngulo = 0;

    if (inputCommand == "*meio#")
      voiceAngulo = 90;

    if (inputCommand == "*ligar falante#")
      falanteLigado = true;

    if (inputCommand == "*desligar falante#")
      falanteLigado = false;

    if (inputCommand == "*ligar lâmpada#")
      lampadaLigada = true;

    if (inputCommand == "*desligar lâmpada#")
      lampadaLigada = false;

    if (inputCommand == "*ligar sensor#")
      lampadaSensor = true;

    if (inputCommand == "*desligar sensor#")
      lampadaSensor = false;


    inputCommand = "";
    inputComplete = false;
    
  }
  
  if (voiceAngulo < 0) voiceAngulo = 0;
  if (voiceAngulo > 179) voiceAngulo = 179;
  
  if (blinkLed) //esse esquema é para piscar o led sem colocar delay.
  {
    if (blinkInterval < 100) blinkInterval = 100;
    
    if(currentMillis - previousMillis >= blinkInterval) {
      // a última vez q fez o switch do led
      previousMillis = currentMillis;   
  
      // inverte o led:
      if (ledState == LOW)
        ledState = HIGH;
      else
        ledState = LOW;
  
      digitalWrite(led, ledState);
    }
    
  }
  
  if (potLigado)
    motor.write(potAngulo);
  else
    motor.write(voiceAngulo);
  
  if (falanteLigado)
  {
    int pitch = map(lightValue,lightLow,lightHigh, 50, 4000);
    tone(falante, pitch, 20);
  }
  
  if (lampadaSensor)
  {
    if (lightValue > lightMiddle)
      digitalWrite(lampada, LOW);
    else
      digitalWrite(lampada, HIGH);
  }
  else
  {
    if (lampadaLigada)
      digitalWrite(lampada, HIGH);
    else
      digitalWrite(lampada, LOW);
  }
}
