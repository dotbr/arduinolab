/**
 * Ubalab Robot Project
 *
 * Name: robot.ino
 * Purpose: learning and encourage students to know basics arduino skills.
 * Last update: 26/11/2015
 *
 * Hardware:
 *    - Arduino Uno r3
 *    - Micro Servo Motor SG90
 *    - HC-SR04 Ultrasonic Sensor
 *    - L293D Driver Four H-Bridge
 *    - Triple Axis Magnetometer - HMC5883
 *    - Kit 2 Wheels
 *    - 2 Celular Batteries
 *    - Buzzer
 *
 * Libraries:
 *    - Adafruit HMC5883 Unified by Adafruit Version 1.0.0
 *    - Adafruit Motor Shield by Adafruit Version 1.0.0
 *    - Adafruit Unified Sensor by Adafruit Version 1.0.2
 *    - NewPing
 */

#include <NewPing.h>
#include <AFMotor.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// Assign a unique ID to this sensor at the same time
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

#define BEEP 16

#define TRIGGER_PIN  15  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     14  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

AF_DCMotor motorE(4); 
AF_DCMotor motorD(1);

boolean indoPraFrente = false;
boolean som = false;
boolean obstaculo = false;

const int margemAngulo = 3;
int anguloAlvo = 0;
int anguloAtual = 0;

const int VEL = 180;

int velocidadeE = 0;
int velocidadeD = 0;

void displaySensorDetails(void) {
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/* Ultrasonic Sensor - Retorna distancia obstáculo em centímetros */
  int verificarDistancia(){
  
  int cm = sonar.ping_cm();
  Serial.print("CM: ");
  Serial.println(cm);  
  return  (cm == 0 ? MAX_DISTANCE : cm); 
}

void andarPraFrente(){
  //if (!indoPraFrente) {
    motorE.setSpeed(velocidadeE);   //Define a velocidade maxima
    motorE.run(FORWARD);            //Gira o motor sentido horario
    motorD.setSpeed(velocidadeD);   //Define a velocidade maxima
    motorD.run(FORWARD);            //Gira o motor sentido horario
    indoPraFrente = true;
    delay(100);
  //}
}

void darRe(int tempo){
  //if (!indoPraFrente) {
    motorE.setSpeed(VEL);   //Define a velocidade maxima
    motorE.run(BACKWARD);   //Gira o motor sentido horario
    motorD.setSpeed(VEL);   //Define a velocidade maxima
    motorD.run(BACKWARD);   //Gira o motor sentido horario
    //indoPraFrente = true;
    delay(tempo);
    parar();
  //}
}

void parar(){
  //if (indoPraFrente) {
    motorE.setSpeed(0);
    motorE.run(RELEASE);
    motorD.setSpeed(0);
    motorD.run(RELEASE);
    indoPraFrente = false;
  //}
}

void virarDireita() {
  motorE.setSpeed(VEL); 
  motorE.run(FORWARD);
  motorD.setSpeed(VEL); 
  motorD.run(BACKWARD);
  delay(400);
  parar();  
}

void setup() {

  Serial.begin(9600);
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
  pinMode(BEEP,OUTPUT);

  /* Initialise the sensor */
  if(!mag.begin())
  {
    // There was a problem detecting the HMC5883 ... check your connections
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

  // Display some basic information on this sensor
  displaySensorDetails();
  
  velocidadeE = VEL;
  velocidadeD = VEL;
  
  anguloAlvo = pegarBussola();
}

void loop() {
  if (!som) {
    analogWrite(BEEP, 255);
    som = true;
  } else {
    analogWrite(BEEP, 0);
    som = false;
  }
  
  anguloAtual = pegarBussola();
  calcDesiredTurn();

  //if (verificarDistancia() < 25) {
    if (verificarDistancia() < 25) {
      if (verificarDistancia() < 25) obstaculo = true;
      else obstaculo = false;
    } else obstaculo = false;
  //} else obstaculo = false;

  if (obstaculo) {
    //virarDireita
    parar();
    delay(200);
    darRe(500); 
    delay(500);
    virarDireita();
    delay(500);
    anguloAlvo = pegarBussola();
  } else {
    //andarPraFrent
    andarPraFrente();
  }
  delay(100);
}

int pegarBussola() {
  // Get a new sensor event
  sensors_event_t event;
  mag.getEvent(&event);

  // Display the results (magnetic vector values are in micro-Tesla (uT))
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.x, event.magnetic.y); //referência é o eixo y!!

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = -0.38; // como mudou a referência, utilizamos a declinação negativa.

  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI;

  Serial.print("Heading (degrees): ");
  Serial.println(headingDegrees);
  return (int)headingDegrees;
}

void calcDesiredTurn(void)
{
  // calculate where we need to turn to head to destination
  int erroAtual = anguloAlvo - anguloAtual;

  // adjust for compass wrap
  if (erroAtual < -180)
    erroAtual += 360;
  if (erroAtual > 180)
    erroAtual -= 360;

  // calculate which way to turn to intercept the targetHeading
  // if within tolerance, don't turn
  if (abs(erroAtual) <= margemAngulo) {      
    velocidadeD = VEL;
    velocidadeE = VEL;
  }
  else if (erroAtual < 0) {
    velocidadeE = VEL - 30;
    velocidadeD = VEL;
  }
  else if (erroAtual > 0) {
    velocidadeD = VEL - 30;
    velocidadeE = VEL;
  }
  else {
    velocidadeD = VEL;
    velocidadeE = VEL;
  }
}
