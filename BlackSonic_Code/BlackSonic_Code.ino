#include <QTRSensors.h>
QTRSensors qtr;

#define Calibration_Time   2000  

#define LED1          21
#define LED2          14

#define STBY          8

//Para el motor A
#define PWMA          5
#define AIN1          7
#define AIN2          6

//Para el motor B
#define PWMB          11
#define BIN1          9
#define BIN2          10

const uint8_t Number_Sensors = 6;     //numero de sensores usados
uint16_t sensorValues[Number_Sensors];

//Variables para el PID
float Kp=0;
float Kd=0;
float Ki=0;

unsigned long TiempoAhora = 0;

int periodo = 250;                    
int velocidad=135;

int Proporcional=0;
int derivativo=0;
int integral=0;
int salida_pwm=0;
int proporcional_pasado=0;
int position=0;
int position_print=0;

int motor_adelante=0;
int motor_atras=0;


void Adelante(){
  digitalWrite(STBY, LOW);
  analogWrite(PWMA, 135);
  analogWrite(PWMB, 135);
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void Atras(){
  digitalWrite(STBY, LOW);
  analogWrite(PWMA, 135);
  analogWrite(PWMB, 135);
  digitalWrite(AIN2, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void Izquierda(){
  digitalWrite(STBY, LOW);
  analogWrite(PWMA, motor_atras);
  analogWrite(PWMB, motor_adelante);
  digitalWrite(AIN2, LOW);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void Derecha(){
  digitalWrite(STBY, LOW);
  analogWrite(PWMA, motor_adelante);
  analogWrite(PWMB, motor_atras);
  digitalWrite(AIN2, LOW);
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void Stop(){
  digitalWrite(STBY,  HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void pid(int velocidad, float Kp, float Ki, float Kd){

  uint16_t position = qtr.readLineBlack(sensorValues);

  Proporcional = (position) - 2500;

  if (Proporcional<0){
    Izquierda();
  }

  else if (Proporcional>0){
    Derecha();
  }

  integral += Proporcional;

  derivativo = (Proporcional - proporcional_pasado);

  if (integral>1000) integral=1000;
  if (integral<-1000) integral=-1000;

   salida_pwm = ( Proporcional * Kp ) + ( derivativo * Kd ) + (integral * Ki);

  if (  salida_pwm > velocidad )  salida_pwm = velocidad; 
  if ( salida_pwm < -velocidad )  salida_pwm = -velocidad;

  motor_adelante = (salida_pwm + velocidad);
  motor_atras = (salida_pwm - velocidad);

  if (motor_adelante > 255) motor_adelante = 255;
  if (motor_atras < 0) motor_atras = 0;

  proporcional_pasado = Proporcional;

}

void setup() {

  pinMode(STBY, OUTPUT);  
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A1, A2, A3, A4, A5, A6}, Number_Sensors);

  delay(500);

  //Se prende el led1 indicando la etapa de calibracion.
  digitalWrite(LED1, HIGH);

   for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }

  digitalWrite(LED1, LOW);
  //Se apaga el led 1 y se prende el led2 indicando que ya termino la etapa de calibracion.
  digitalWrite(LED2, HIGH);

}

void loop() {

analogWrite(PWMA, velocidad);
analogWrite(PWMB, velocidad);

  for (uint8_t i = 0; i < Number_Sensors; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  if(millis() > TiempoAhora + periodo)
  {
    TiempoAhora = millis();
    position_print = qtr.readLineBlack(sensorValues);
    Serial.println(position_print);
  }

  pid(velocidad,Kp,Ki,Kd);

}
