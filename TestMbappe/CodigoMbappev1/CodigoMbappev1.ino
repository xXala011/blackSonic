#include <QTRSensors.h>
//_--BlackSonic Code--_//

//Definiendo Salidas de los motores
#define motorAInput1 7
#define motorAInput2 6
#define motorBInput1 9
#define motorBInput2 10
#define motorAPWM 5
#define motorBPWM 11
#define STBY      8 // Pin STBY del TB6612FNG
#define LEDCC 4
QTRSensors qtr;
//Definiendo Constantes para PID
int KP=1;
int KI=0;
int KD=4;
int lastError = 0;
int integral = 0;

//Definiendo cantidad de sensores
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];


void setup() {
 //Definiendo Salidas 
 pinMode(motorAInput1, OUTPUT);
 pinMode(motorAInput2, OUTPUT);
 pinMode(motorBInput1, OUTPUT);
 pinMode(motorBInput2, OUTPUT);
 pinMode(motorAPWM, OUTPUT);
 pinMode(motorBPWM, OUTPUT);
 pinMode(STBY, OUTPUT);
 digitalWrite(STBY, HIGH);
 
 //Definiendo el tipo de sensores y declarando salidas donde estos se encuentran
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A1, A2, A3, A4, A5, A6}, SensorCount);
  qtr.setEmitterPin(A0);
 //Calibracion
 digitalWrite(LED_BUILTIN, HIGH);
   for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  } 
  digitalWrite(LED_BUILTIN, LOW);
  delay(5000);//Delay de 5seg para acomodar en la linea a Mbappe
digitalWrite(LEDCC,HIGH);
delay(700);
}

void loop() {

uint16_t position = qtr.readLineBlack(sensorValues); //Definiendo el tipo de linea que va a recorrer

 //Calculo para determinar el centro de control
  int weightedSum = 0;
  int sum = 0;
  for (int i = 0; i < SensorCount; i++) {
    weightedSum += (i * sensorValues[i]);
    sum += sensorValues[i];
  }
  int error = position - (SensorCount - 1) * 1000 / 2;
 //Variables para calcular el PID
 int proportional = KP * error;
 int derivative = KD * (error - lastError);
 int integral =+ error;
 int integralComponent = KI * integral;

 //Calibracion de motores
 int controlSignal = proportional + derivative + integralComponent;
 int motorSpeed=90;
 int motorSpeedA = motorSpeed - controlSignal;
 int motorSpeedB = motorSpeed + controlSignal;
 motorSpeedA = constrain(motorSpeedA, 0, 255);
 motorSpeedB = constrain(motorSpeedB, 0, 255);
 //Declarando salidas del driver de motores (Adelante)
  digitalWrite(motorAInput1, 0);
  digitalWrite(motorAInput2, 1); 
  digitalWrite(motorBInput1, 1);
  digitalWrite(motorBInput2, 0);
  analogWrite(motorAPWM, motorSpeedA);
  analogWrite(motorBPWM, motorSpeedB);
 //Variable para detectar el ultimo error de control PID
  int lastError = error;

}
