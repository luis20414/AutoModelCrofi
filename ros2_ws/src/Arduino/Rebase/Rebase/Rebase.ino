#include <Servo.h>

Servo servo;
Servo esc;
const int pinESC = 9;
const int pinSERVO = 5;

float angulo = 90;

//  90 grados: llantas derechas
//  50 grados: Maximo a la izquierda
//  130 grados: Maximo a la izquierda

int adelante = 1545;
int freno = 1500;
int reversa = 1390;

void setup() {
  Serial.begin(115200);
  esc.attach(pinESC);
  servo.attach(pinSERVO);
  Serial.println("Esperando Inicializacion del Driver");
  delay(5000);
  esc.writeMicroseconds(1500); // Neutral
  servo.write((int)angulo);
  Serial.println("Iniciando PROGRAMA");
}

void loop() {

  recibirMotor();
  
  //rebase();
  //delay(1000000);
}

void calibracion(){
  Serial.println("Iniciando calibracion");
  Serial.println("2000");
  esc.writeMicroseconds(2000); 
  delay(500);                 
  Serial.println("1000");    
  esc.writeMicroseconds(1000);   
  delay(1000);
  Serial.println("1500 (neutro)");
  esc.writeMicroseconds(freno);
}

void rebase(){
  giroGrados(50);
  avanzar(1200);
  giroGrados(130);
  avanzar(600);
  giroGrados(90);
  delay(200);
  giroGrados(130);
  avanzar(100);
  giroGrados(50);
  retroceder(300);
}

void avanzar(int tiempo){
  avanzar(tiempo, adelante);
}

void avanzar(int tiempo, int velocidad){
  if (velocidad > 1500){
    esc.writeMicroseconds(adelante); // Avanza por cierto tiempo
    delay(tiempo);
    esc.writeMicroseconds(freno); //Regresa a neutro
  }
}

void retroceder(int tiempo){
  retroceder(tiempo, reversa);
}

void retroceder(int tiempo, int velocidad){
  esc.writeMicroseconds(velocidad); // 
  delay(100);                       // Estas 4 lineas son necesarias. Sin ellas, el
  esc.writeMicroseconds(freno);     // motor no retrocede por alguna razon -_-
  delay(100);                       //

  esc.writeMicroseconds(velocidad); // Retrocede por cierto tiempo
  delay(tiempo);
  esc.writeMicroseconds(freno); //Regresa a neutro
}

void giroRadianes(float angulo){ //Recibe un angulo en RADIANES
  float targetAngle = 90 + (angulo * 57.2957); // Conversion a grados y desplazamiento de unidades
  targetAngle = constrain(targetAngle, 50, 130); // Limitamos el angulo a un cierto rango
  servo.write(int(targetAngle));
}

void giroGrados(float angulo){
  float targetAngle = constrain(angulo, 50, 130); // Limitamos el angulo a un cierto rango
  servo.write(int(targetAngle));
}

void recibirMotor(){
  //Recibir datos
  if (Serial.available() > 6) {  // "motor XXXX" son 10 caracteres
    Serial.println("Dato Recibido");
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.startsWith("motor ")) {
      int pwmValue = input.substring(6).toInt();
      pwmValue = constrain(pwmValue, 1300, 1600);
      Serial.println(pwmValue);
      if (pwmValue < 1500){
        Serial.println("Reversa");
        esc.writeMicroseconds(pwmValue);
        delay(100);
        esc.writeMicroseconds(freno);
        delay(100);
        esc.writeMicroseconds(pwmValue);
      }
      esc.writeMicroseconds(pwmValue);
      return;
    }
  }
}