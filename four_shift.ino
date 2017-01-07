//Pin connected to ST_CP of 74HC595---
int latchPin = 10;
//Pin connected to SH_CP of 74HC595
int clockPin = 13;
////Pin connected to DS of 74HC595
int dataPin = 11;

void setup() {
  //set pins to output because they are addressed in the main loop
  pinMode(latchPin, OUTPUT);
  Serial.begin(9600);

   //function that blinks all the LEDs
  //gets passed the number of blinks and the pause time
  blinkAll(2,500); 
}

unsigned long ledData = 0;

void loop() {

  for(int numDelay = 10; numDelay < 70; numDelay = numDelay * 2)
  {
      FillInToCenter(numDelay); //OK
      FillOutFromCenter(numDelay);
      ledShiftLeft(numDelay, 1, 0);
      ledShiftRight(numDelay, 1, 0);
      FillInFromCenter(numDelay); //OK
      FillOutToCenter(numDelay); //OK
      blinkAll_3Bytes(6,numDelay); 
      FillInFromRight(numDelay);
      ledShiftLeft(numDelay, 1, 1);
      FillOutFromLeft(numDelay);
      FillInFromLeft(numDelay);
      ledShiftRight(numDelay, 1, 1);
      FillOutFromRight(numDelay); 
      ledShiftFromCenter(numDelay);
      ledShiftToCenter(numDelay);
      for(int i = 0; i < 6; i++)
      {
        shiftOutLong(dataPin, clockPin, 0xFF00FF00);
        delay(numDelay*6);
        shiftOutLong(dataPin, clockPin, 0x00FF00FF);
        delay(numDelay*6);
      }
      for(int i = 0; i < 6; i++)
      {
        shiftOutLong(dataPin, clockPin, 0xF0F0F0F0);
        delay(numDelay*6);
        shiftOutLong(dataPin, clockPin, 0x0F0F0F0F);
        delay(numDelay*6);
      }
      for(int i = 0; i < 6; i++)
      {
        shiftOutLong(dataPin, clockPin, 0xCCCCCCCC);
        delay(numDelay*6);
        shiftOutLong(dataPin, clockPin, 0x33333333);
        delay(numDelay*6);
      }
      for(int i = 0; i < 6; i++)
      {
        shiftOutLong(dataPin, clockPin, 0xAAAAAAAA);
        delay(numDelay*6);
        shiftOutLong(dataPin, clockPin, 0x55555555);
        delay(numDelay*6);
      }
      
  }

  for(int numDelay = 70; numDelay > 10; numDelay = numDelay / 2)
  {
      FillInToCenter(numDelay); //OK
      FillOutFromCenter(numDelay);
      ledShiftLeft(numDelay, 1, 0);
      ledShiftRight(numDelay, 1, 0);
      FillInFromCenter(numDelay); //OK
      FillOutToCenter(numDelay); //OK
      blinkAll_3Bytes(6,numDelay); 
      FillInFromRight(numDelay);
      ledShiftLeft(numDelay, 1, 1);
      FillOutFromLeft(numDelay);
      FillInFromLeft(numDelay);
      ledShiftRight(numDelay, 1, 1);
      FillOutFromRight(numDelay); 
      ledShiftFromCenter(numDelay);
      ledShiftToCenter(numDelay);
      for(int i = 0; i < 6; i++)
      {
        shiftOutLong(dataPin, clockPin, 0xFF00FF00);
        delay(numDelay*6);
        shiftOutLong(dataPin, clockPin, 0x00FF00FF);
        delay(numDelay*6);
      }
      for(int i = 0; i < 6; i++)
      {
        shiftOutLong(dataPin, clockPin, 0xF0F0F0F0);
        delay(numDelay*6);
        shiftOutLong(dataPin, clockPin, 0x0F0F0F0F);
        delay(numDelay*6);
      }
      for(int i = 0; i < 6; i++)
      {
        shiftOutLong(dataPin, clockPin, 0xCCCCCCCC);
        delay(numDelay*6);
        shiftOutLong(dataPin, clockPin, 0x33333333);
        delay(numDelay*6);
      }
      for(int i = 0; i < 6; i++)
      {
        shiftOutLong(dataPin, clockPin, 0xAAAAAAAA);
        delay(numDelay*6);
        shiftOutLong(dataPin, clockPin, 0x55555555);
        delay(numDelay*6);
      }

  }  
 // 0000 0000 0000 0000 0000 0000
 // 0000 0000 0001 1000 0000 0000
 // 0000 0000 0011 1100 0000 0000
 // 0000 0000 0111 1110 0000 0000
 // 0000 0000 1111 1111 0000 0000
 // 0000 0001 1111 1111 1000 0000
 // 0000 0011 1111 1111 1100 0000
 // 0000 0111 1111 1111 1110 0000
 // 0000 1111 1111 1111 1111 0000
 // 0001 1111 1111 1111 1111 1000
 // 0011 1111 1111 1111 1111 1100
 // 0111 1111 1111 1111 1111 1110
 // 1111 1111 1111 1111 1111 1111
 // 0111 1111 1111 1111 1111 1110
 // 0011 1111 1111 1111 1111 1100
 // 0001 1111 1111 1111 1111 1000
 // 0000 1111 1111 1111 1111 0000
 // 0000 0111 1111 1111 1110 0000
 // 0000 0011 1111 1111 1100 0000
 // 0000 0001 1111 1111 1000 0000
 // 0000 0000 1111 1111 0000 0000
 // 0000 0000 0111 1110 0000 0000
 // 0000 0000 0011 1100 0000 0000
 // 0000 0000 0001 1000 0000 0000

}

void FillOutFromRight(int numDelay)
{
  Serial.println("FillOutFromRight");

  ledData = 0xFFFFFFFF;
  
  for(int i = 0; i < 32; i++)
  {
    ledData = ledData ^ 1UL<<i;
    Serial.print(i);
    Serial.print(" = ");
    shiftOutLong(dataPin, clockPin, ledData);
    delay(numDelay/2);
  }
}

void FillOutFromLeft(int numDelay)
{
  Serial.println("FillOutFromLeft");

  ledData = 0xFFFFFFFF;
  
  for(int i = 0; i < 32; i++)
  {
    ledData = ledData ^ 0x80000000>>i;
    Serial.print(i);
    Serial.print(" = ");
    shiftOutLong(dataPin, clockPin, ledData);
    delay(numDelay/2);
  }
}

void FillInFromRight(int numDelay)
{
  Serial.println("FillInFromRight");

  ledData = 0;
  
  for(int i = 0; i < 32; i++)
  {
    ledData = ledData | 1UL<<i;
    Serial.print(i);
    Serial.print(" = ");
    shiftOutLong(dataPin, clockPin, ledData);
    delay(numDelay/2);
  }
}

void FillInFromLeft(int numDelay)
{
  Serial.println("FillInFromLeft");

  ledData = 0;
  
  for(int i = 0; i < 32; i++)
  {
    ledData = ledData | 0x80000000>>i;
    Serial.print(i);
    Serial.print(" = ");
    shiftOutLong(dataPin, clockPin, ledData);
    delay(numDelay/2);
  }
}


void FillInFromCenter(int numDelay)
{
  Serial.println("FillInFromCenter");

  ledData = 0;
  
  for(int i = 0; i < 16; i++)
  {
    ledData = ledData | 0x8000UL>>i | 0x10000UL<<i;
    Serial.print(i);
    Serial.print(" = ");
    shiftOutLong(dataPin, clockPin, ledData);
    delay(numDelay);
  }
}

void FillOutFromCenter(int numDelay)
{
  Serial.println("FillOutFromCenter");

  ledData = 0xFFFFFFFF;
  
  for(int i = 0; i < 16; i++)
  {
    ledData = ledData ^ ((0x8000UL>>i) + (0x10000UL<<i));
    Serial.print(i);
    Serial.print(" = ");
    shiftOutLong(dataPin, clockPin, ledData);
    delay(numDelay);
  }

}

void FillInToCenter(int numDelay)
{
  Serial.println("FillInToCenter");

  ledData = 0;
  
  for(int i = 15; i >= 0; i--)
  {
    ledData = ledData | 0x8000UL>>i | 0x10000UL<<i;
    Serial.print(i);
    Serial.print(" = ");
    shiftOutLong(dataPin, clockPin, ledData);
    delay(numDelay);
  }

}

void FillOutToCenter(int numDelay)
{
  Serial.println("FillOutToCenter");

  ledData = 0xFFFFFFFF;
  
  for(int i = 15; i >=0; i--)
  {
    ledData = ledData ^ ((0x8000UL>>i) + (0x10000UL<<i));
    Serial.print(i);
    Serial.print(" = ");
    shiftOutLong(dataPin, clockPin, ledData);
    delay(numDelay);
  }

}

void ledShiftLeft(int numDelay, unsigned long data, boolean inverted)
{
  Serial.println("ledShiftLeft");

    for (int j = 0; j < 32; j++) {
    ledData = data<<j;
    if (inverted) ledData = ~ledData;
    Serial.print(j);
    Serial.print(" = ");
    shiftOutLong(dataPin, clockPin, ledData);
    delay(numDelay/2);
  }
}

void ledShiftRight(int numDelay, unsigned long data, boolean inverted)
{
  Serial.println("ledShiftRight");

    for (int j = 31; j >= 0; j--) {
    ledData = data<<j;
    if (inverted) ledData = ~ledData;
    Serial.print(j);
    Serial.print(" = ");
    shiftOutLong(dataPin, clockPin, ledData);
    delay(numDelay/2);
  }
}

void ledShiftFromCenter(int numDelay)//, unsigned long data, boolean inverted)
{
  Serial.println("ledShiftFromCenter");

  ledData = 0;
  
  for(int i = 0; i < 16; i++)
  {
    ledData = 0x8000UL>>i | 0x10000UL<<i;
    Serial.print(i);
    Serial.print(" = ");
    shiftOutLong(dataPin, clockPin, ledData);
    delay(numDelay);
  }
}


void ledShiftToCenter(int numDelay)//, unsigned long data, boolean inverted)
{
  Serial.println("ledShiftToCenter");

  ledData = 0;
  
  for(int i = 0; i < 16; i++)
  {
    ledData = 0x80000000UL>>i | 0x1UL<<i;
    Serial.print(i);
    Serial.print(" = ");
    shiftOutLong(dataPin, clockPin, ledData);
    delay(numDelay);
  }
}

// the heart of the program
void shiftOutLong(int myDataPin, int myClockPin, unsigned long myDataOut) {
  // This shifts 8 bits out MSB first, 
  //on the rising edge of the clock,
  //clock idles low

  //internal function setup
  int i=0;
  int pinState;
  pinMode(myClockPin, OUTPUT);
  pinMode(myDataPin, OUTPUT);

  //ground latchPin and hold low for as long as you are transmitting
  digitalWrite(latchPin, 0);

  //clear everything out just in case to
  //prepare shift register for bit shifting
  digitalWrite(myDataPin, 0);
  digitalWrite(myClockPin, 0);

  Serial.println(myDataOut);

  //for each bit in the byte myDataOut�
  //NOTICE THAT WE ARE COUNTING DOWN in our for loop
  //This means that %00000001 or "1" will go through such
  //that it will be pin Q0 that lights. 
  for (i=31; i>=0; i--)  {
    digitalWrite(myClockPin, 0);

    //if the value passed to myDataOut and a bitmask result 
    // true then... so if we are at i=6 and our value is
    // %11010100 it would the code compares it to %01000000 
    // and proceeds to set pinState to 1.
    if ( myDataOut & (1UL<<i) ) {
      pinState= 1;
    }
    else {	
      pinState= 0;
    }

    //Sets the pin to HIGH or LOW depending on pinState
    digitalWrite(myDataPin, pinState);
    //register shifts bits on upstroke of clock pin  
    digitalWrite(myClockPin, 1);
    //zero the data pin after shift to prevent bleed through
    digitalWrite(myDataPin, 0);
  }

  //stop shifting
  digitalWrite(myClockPin, 0);

  //Serial.println(ledData);
  //return the latch pin high to signal chip that it 
  //no longer needs to listen for information
  digitalWrite(latchPin, 1);

}


//blinks the whole register based on the number of times you want to 
//blink "n" and the pause between them "d"
//starts with a moment of darkness to make sure the first blink
//has its full visual effect.
void blinkAll(int n, int d) {
  shiftOutLong(dataPin, clockPin, 0);
  delay(200);
  for (int x = 0; x < n; x++) {
    shiftOutLong(dataPin, clockPin, 0xFFFFFFFFUL);
    delay(d);
    shiftOutLong(dataPin, clockPin, 0);
    delay(d);
  }
}
