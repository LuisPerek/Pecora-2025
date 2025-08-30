#include <dummy.h>
#include "DRV8833.h"
#include <QTRSensors.h>

// Pinos dos motores
DRV8833 driver = DRV8833();
const int inputR1 = 19, inputR2 = 18;
const int inputL1 = 16, inputL2 = 17;
// Sensores
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

uint16_t position;
  
   // Base de velocidade
  int baseSpeed = 65;
  int turnSpeed = 130;
  int turnSpeedM = 200;
  int cavalo;

  // Verifica posição da linha
  int error;
  int leftSpeed;
  int rightSpeed;
// Funções auxiliares para controle do motor

//sensores laterais
boolean final = false, ligado = true, ligado2 = true, virar = false;
int rightSensor;
int leftSensor;
int errorMax = 2800, errorMin = 600;
int marcaFinal = -200, marcaVirar = 0;
int marcaDirecao;


void setup() {
  Serial.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){22, 14, 27, 26, 25, 33, 32, 23}, SensorCount);
  qtr.setEmitterPin(13);
  pinMode(LED_BUILTIN, OUTPUT);
  driver.attachMotorB(inputL1, inputL2);
  driver.attachMotorA(inputR1, inputR2);

  digitalWrite(LED_BUILTIN, HIGH);
  for (int i = 0; i < 200; i++) { // 400 iterações para calibrar
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
  delay(3000);
}
  
void loop() {

  while(ligado == true){
    andar();
    finalizar();
  }
  
  while(marcaFinal < 150){
    baseSpeed = 75;
    turnSpeed = 75;
    errorMin = 500;
    andar();
    marcaFinal++;
    
  }
  if (ligado2){
    driver.motorBForward(-30);
    driver.motorAStop();
  }
  pass();
  
  

}

void andar(){
  position = qtr.readLineWhite(sensorValues);  

   // Base de velocidade
  baseSpeed = baseSpeed;

  // Verifica posição da linha
  error = position - 3500; // 2500 é o centro ideal
  leftSpeed = -1 * baseSpeed;
  rightSpeed = baseSpeed;

  
  if(error > 3495 || error < (-3495)){
    if(marcaDirecao>=0){
      leftSpeed = -10;
      rightSpeed = turnSpeed;
    }else{
      leftSpeed = -turnSpeed;
      rightSpeed = 10;
    }
  }else if (error > errorMax){
    leftSpeed = -10;
    rightSpeed = turnSpeedM;
    marcaDirecao += 1;
  }else if(error < (-errorMax)){
    leftSpeed = -turnSpeedM;
    rightSpeed = 10;
    marcaDirecao -= 1;
  }else if (error > errorMin) { // Linha está à direita
    leftSpeed = -10;
    rightSpeed = turnSpeed;
    marcaDirecao += 1;

  }else if (error < (-errorMin)) { // Linha está à direita
    leftSpeed = -turnSpeed;
    rightSpeed = 10;
    marcaDirecao -= 1;
  }

  driver.motorAForward(rightSpeed);
  driver.motorBForward(leftSpeed);

}

void finalizar(){
  rightSensor = analogRead(15);

  if (sensorValues[0] < 600 && sensorValues[7] < 600){
    marcaFinal = 0;
    marcaVirar = -10;
  }

  if (rightSensor < 1600 && marcaFinal > 100){
    ligado = false;
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
    marcaFinal = 0;
  }
  marcaFinal++;
}


void pass(){}
