#include <QTRSensors.h>
//INCLUIMOS LA LIBRERIA DE LOS SENSORES QTR
#include <QTRSensors.h>
//DECLARAMOS ENTRADAS PARA EL PUENTE H AL ARDUINO

// MOTOR DERECHO
const int PWMA = 9; // ~
const int AIN2 = 8; // (+)
const int AIN1 = 7; // (-)
// MOTOR IZQUIERDO
const int PWMB = 3; // ~
const int BIN2 = 4; // (-)
const int BIN1 = 5; // (+)

const int STANDBY  = 6;
///DECLARAMOS SALIDA PARA SEÑALIZAR EL CALIBRADO
const int LED = 13;
//VARIABLES DE ALMACENAMIENTO
int P = 0;
int I = 0;
int D = 0;
int LAST = 0;
float vel;
//DECLARAMOS VELOCIDAD DEL ROBOT OJO LLEGA HASTA 255
int VelMax = 83;
//////////////////////////////////////////////////////////////////////////////////////////
//constantes
//float Kp = 0.094519; int Kd = 0.955; int Ki = 0.000151;
//float Kp = 0.0901; int Kd = 0.67; int Ki = 0.0105;
float Kp = 0.0901 ; int Kd = 1.0; int Ki = 0.00105;
/////////////////////////////////////////////////////////////////////////////////
#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  1  // average 4 analog samples per sensor reading
#define EMITTER_PIN             11  // emitter is controlled by digital pin 2

// 
QTRSensorsAnalog qtra((unsigned char[]) {A7, A6, A5, A4, A3, A2, A1 , A0}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

unsigned int position = 0;

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(STANDBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  delay(1000);
  //prendemos el led (LED, HIGH);
  for (int j = 0; j < 150; j++) {
    digitalWrite(LED, HIGH);
    delay(20);
    qtra.calibrate();            // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
    digitalWrite(LED, LOW);   // turn off LED
    delay(20);
  }
  //Serial.begin(9600);
  //ENCENDEMOS EL LED
  digitalWrite(LED, HIGH);
  
  while (true) {
    delay(100);

      //HABILITAMOS EL PUENTE H PARA EL FUNCIONAMIENTO DEL MISMO
      //INICIAMOS EL ARRANQUE
      digitalWrite(STANDBY, HIGH);
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      analogWrite(PWMA, 0); // 0
      analogWrite(PWMB, 0); // 0
      delay(100);
      
      break; //saltamos hacia el bucle principal
    }
  }
  

void loop() {

  //LEEMOS LA SEÑAL DE LOS SENSORES
  qtra.read(sensorValues);
  position = qtra.readLine(sensorValues, QTR_EMITTERS_ON, 0); // 0 pista blanca, linea negra

  P = ((position) - (3500)); /// ERROR

  //Serial.println(P);
  //delay(100);

  /////FRENOS////
  if (P <= -3460) {   //debemos girar el motor derecha adelante
    analogWrite(PWMA, 255); // VELOCIDAD PARA EL MOTOR DERECHO 200
    analogWrite(PWMB, 180); //  VELOCIDAD PARA EL MOTOR IZQUIERDO 255
    izquierda();
    digitalWrite(LED, HIGH); // encendemos LED en señal de que esta en una curva a la izquierda
  } else {
    if (P >= 3460) {    // debemos girar el motor izquierdo adelante
      analogWrite(PWMA, 180); // VELOCIDAD PARA EL MOTOR DERECHO 255
      analogWrite(PWMB, 255); //  VELOCIDAD PARA EL MOTOR IZQUIERDO 200
      derecha();
      digitalWrite(LED, HIGH); // encendemos LED en señal de que esta en una curva a la derecha
    } else {
      digitalWrite(LED, LOW);
      D = (P - LAST); /// ERROR MENOS EL ERROR ANTERIOR , DERIVATIVO
      I = (P + LAST); //INTEGRAL
      
      vel = ( P * Kp ) + ( D * Kd ) + ( I * Ki ); ///VELOCIDAMAX =VELOCIDAD PUNTA , V

      //Serial.println(vel);

      if (vel > VelMax) {
        vel = VelMax;
      }
      if (vel < -VelMax) {
        vel = -VelMax;
      }
     
      analogWrite(PWMA, (VelMax - vel)); // VELOCIDAD PARA EL MOTOR DERECHO (-)
      analogWrite(PWMB, (VelMax + vel)); //  VELOCIDAD PARA EL MOTOR IZQUIERDO (+)
      /*
        Serial.print(VelIzq); //inzquierdo
        Serial.print("-");
        Serial.println(VelDer); //derecho
        Serial.println();
        delay(1000);
      */
      adelante();
      LAST = P; // ERROR ANTERIOR
    }
  }
}//BUCLE INFINITO

void adelante(){
  digitalWrite(AIN1, LOW);  ///FRENTE
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); ///FRENTE
  digitalWrite(BIN2, LOW); 
}
void izquierda(){
  digitalWrite(AIN1, LOW);  // FRENTE
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW); // ATRAS
  digitalWrite(BIN2, HIGH);
}
void derecha(){
  digitalWrite(AIN1, HIGH);  // ATRAS
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); // FRENTE
  digitalWrite(BIN2, LOW); 
}
void atras(){
  digitalWrite(AIN1, HIGH);  // ATRAS
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); // ATRAS
  digitalWrite(BIN2, HIGH); 
}
/*
   1 Poner las constantes Kp, Ki y Kd a 0
   2 Ir subiendo Kp hasta que el robot empieza a oscilar a la salida de las curvas
   3 Bajar un poco la Kp para que no oscile.
   4 Repetir los pasos 2 y 3 para Ki y Kd
   BAJAR KP PARA QUE NO OSCILE, COMENZAMOS CON KD Y KI EN CERO
*/
