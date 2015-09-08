int led = 13;
int red = 6;
int yellow = 9;
int green = 5;
int blue = 2;
int ledState = LOW;
int redState = LOW;
int yellowState = LOW;
int greenState = LOW;
int blueState = LOW;
unsigned long now;
unsigned long red_previous_time = 0;
unsigned long yellow_previous_time = 0;
unsigned long green_previous_time = 0;
unsigned long blue_previous_time = 0;
unsigned long previous_time = 0;


void setup()
{
  pinMode(led,OUTPUT);
  pinMode(red,OUTPUT);
  pinMode(yellow,OUTPUT);
  pinMode(green,OUTPUT);
  pinMode(blue,OUTPUT);
  Serial.begin(9600);
}

void led_blink(int time)
{
  if (now - previous_time >= time)
  {
    ledState = !ledState;
    digitalWrite(led, ledState);
    previous_time = now;
  }

}

void yellow_blink(int time)
{
  if (now - yellow_previous_time >= time)
  {
    yellowState = !yellowState;
    digitalWrite(yellow, yellowState);
    yellow_previous_time = now;
  }

}

void red_blink(int time)
{
  if (now - red_previous_time >= time)
  {
    redState = !redState;
    digitalWrite(red, redState);
    red_previous_time = now;
  }

}

void green_blink(int time)
{
  if (now - green_previous_time >= time)
  {
    greenState = !greenState;
    digitalWrite(green, greenState);
    green_previous_time = now;
  }

}

void blue_blink(int time)
{
  if (now - blue_previous_time >= time)
  {
    blueState = !blueState;
    digitalWrite(blue, blueState);
    blue_previous_time = now;
  }

}


void loop()
{
  now = millis();
  led_blink(1000);
  yellow_blink(2000);
  red_blink(4000);
  green_blink(8000);
  blue_blink(16000);
}

