    #include <Servo.h>

Servo servo;
int pinServo = 5;
float servoAngle = 90;
const byte numMaxChars = 32;
char recievedChars[numMaxChars];
boolean newData = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Arduino Listo");
  servo.attach(pinServo);
}

void loop() {
  // put your main code here, to run repeatedly:
  static byte index = 0;
  char finalLinea = '\n';
  char rc;
  float goalAngle = 0;

  while (Serial.available() > 0 && newData == false){
    rc = Serial.read();

    if (rc != finalLinea){
      recievedChars[index] = rc;
      index++;
      if (index >= numMaxChars){
        index = numMaxChars - 1;
      }
    }
    else {
    recievedChars[index] = '\0';
    index = 0;
    newData = true;
    }
  }

  goalAngle = atof(recievedChars);
  servoAngle = 90 - (goalAngle * 57.2957);
  if (newData == true){
  Serial.println(servoAngle);
  newData = false;
  servo.write(servoAngle);
  }
}
