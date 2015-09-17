#include <Ultrasonic.h>
#include <AFMotor.h>


#define TRIGGER_PIN  15
#define ECHO_PIN     14

Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);
AF_DCMotor motorE(2); 
AF_DCMotor motorD(3);

boolean indoPraFrente = false;

int verificarDistancia(){
  float cmMsec;
  long microsec = ultrasonic.timing();

  cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);
    
  Serial.print("CM: ");
  Serial.println(cmMsec);  
  
  return (int) cmMsec; 
}

void andarPraFrente(){
  if (!indoPraFrente) {
    motorE.setSpeed(255); //Define a velocidade maxima
    motorE.run(FORWARD);//Gira o motor sentido horario
    motorD.setSpeed(255); //Define a velocidade maxima
    motorD.run(FORWARD);//Gira o motor sentido horario
    indoPraFrente = true;
    delay(500);
  }
}

void parar(){
  if (indoPraFrente) {
    motorE.setSpeed(0); 
    motorE.run(RELEASE);
    motorD.setSpeed(0); 
    motorD.run(RELEASE);
    indoPraFrente = false;
  }
}

void virarDireita(){
  motorE.setSpeed(255); 
  motorE.run(FORWARD);
  motorD.setSpeed(255); 
  motorD.run(BACKWARD);
  delay(500);
  indoPraFrente = true;
  parar();  
}

void setup(){
  
}

void loop(){
  boolean obstaculo;
  
  if (verificarDistancia() < 15) { 
   if (verificarDistancia() < 15) {
    if (verificarDistancia() < 15) obstaculo = true;
   } else obstaculo = false;
  } else obstaculo = false;
  
  if (obstaculo) {
    //virarDireita
   parar(); 
   delay(500);
   virarDireita();
   delay(500);
  } else {
   //andarPraFrent
   andarPraFrente();
  }
 delay(100); 
}
