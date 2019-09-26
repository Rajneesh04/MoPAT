//Robotics Club - Motion Planning System
//Guining Pertin - 05-06-2019
//NodeMCU control code

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <AccelStepper.h>

const char* ssid = "duncan";
const char* password = "11111111";
//Motor driver connections
int enA = 12; //Left
int in1 = 5;
int in2 = 4;
int in3 = 0;
int in4 = 2;
int enB = 14; //Right
int counter = 0;

//LForward - in1 - LOW & in2 - HIGH
//RForward - in3 - LOW & in4 - HIGH

//PID coeffs
float kp_angle = 0.1;
float ki_angle = 5;
float kd_angle = 1000;
float kp_dist = 0.1;
float ki_dist = 15;
float kd_dist = 1000;
//PID values
float currTime_angle = 0;
float prevTime_angle = 0;
float error_angle = 0;
float prevError_angle = 0;
float p_angle = 0;
float i_angle = 0;
float d_angle = 0;
float pid_angle = 0;
float currTime_dist = 0;
float prevTime_dist = 0;
float error_dist = 0;
float prevError_dist = 0;
float p_dist = 0;
float i_dist = 0;
float d_dist = 0;
float pid_dist = 0;
long receivedDistance = 0;//distance to move
long receivedSpeed = 0;
long receivedAcceleration = 0;
char receivedCommand;

bool newData, canMove= false;

AccelStepper stepper1(8, 3, 5, 4, 6);
AccelStepper stepper2(8, 8, 10, 9, 11);

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting..");
  }
  Serial.println("Connected");
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);
  digitalWrite(enA, LOW);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(enB, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW); 
  stepper1.setMaxSpeed(2000);
  stepper1.setAcceleration(1000);
  stepper1.disableOutputs(); 
  stepper2.setMaxSpeed(2000);
  stepper2.setAcceleration(1000);
  stepper2.disableOutputs(); 
}

void loop() {
  String dist;
  String angle;
  String dir;
  if (WiFi.status() == WL_CONNECTED){
    HTTPClient http;
    http.begin("http://192.168.43.135:8080/data.txt");
    int httpCode = http.GET();
  
    if (httpCode > 0) {
      String payload = http.getString();
      Serial.println(payload);  
      dist = payload.substring(0, payload.indexOf('+'));
      angle = payload.substring(payload.indexOf('+')+1, payload.indexOf('-'));
      dir = payload.substring(payload.indexOf('-')+1, payload.indexOf('*'));
      Serial.print(dist);
      Serial.print(",");
      Serial.print(angle);
      Serial.print(",");
      Serial.println(dir);
      if(abs(angle.toInt())>0)
      {
        receivedDistance = 13.66667*(angle.toInt());
        stepper1.enableOutputs();
        stepper2.enableOutputs();
   
          if(dir=='r')
          {
            stepper1.move(receivedDistance);
            stepper2.move(receivedDistance);
          }
          else{
            stepper1.move(-1*receivedDistance);
            stepper2.move(-1*receivedDistance);
          }
          while(abs(stepper1.currentPosition())<receivedDistance && (stepper2.currentPosition())<receivedDistance)
          {
             stepper1.run();
             stepper2.run();
          }
          stepper1.setCurrentPosition(0);
          stepper2.setCurrentPosition(0);
          stepper1.disableOutputs();
          stepper2.disableOutputs();
        }
     
      if(abs(dist.toInt())>0)
      {
        receivedDistance = ((dist.toInt())*4076)/21.98;//perimeter of wheel = 7*pi;number of revolution = dist/(7*pi);number of steps in one full rotation= 4076;
        stepper1.move(-1*receivedDistance);
        stepper2.move(receivedDistance);
        while(abs(stepper1.currentPosition())<receivedDistance && (stepper2.currentPosition())<receivedDistance)
        {
            stepper1.run();
            stepper2.run();
        }
        stepper1.setCurrentPosition(0);
        stepper2.setCurrentPosition(0);
        stepper1.disableOutputs();
        stepper2.disableOutputs();
        
      }
         http.end();
  }
}
