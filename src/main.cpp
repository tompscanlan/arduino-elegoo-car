//www.elegoo.com
#include <Thread.h>
#include <ThreadController.h>
#include <TimerOne.h>
#include <NewPing.h>

#include <Servo.h>  //servo library
volatile Servo myservo;      // create servo object to control servo

int Echo = A4;
int Trig = A5;
#define MAX_DISTANCE 200

NewPing sonar(Trig, Echo, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

#define carSpeed 230
#define safeDistance 80

#define offsetAngle 12 // how far off center is from the actual mount
#define viewAngle 150

#define centerAngle 90 + offsetAngle
#define leftAngle centerAngle + (viewAngle/2)
#define rightAngle centerAngle - (viewAngle/2)

#define viewPorts 3
#define distances[viewPorts-1]

#define shortDelay 200
#define longDelay 1000
#define loopTime 100

volatile int rightDistance = 0, leftDistance = 0, middleDistance = 0;
volatile int servoAngle = 0;
volatile int servoDirection = 30; // 1 = up, -1 is down
int angleBucket = 0;

volatile ThreadController control = ThreadController();
volatile Thread sonarThread = Thread();
volatile Thread moveThread = Thread();


void forward(){ 
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Forward");
}

void back() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Back");
}

void left() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  Serial.println("Left");
}

void right() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Right");
}

void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  Serial.println("Stop!");
} 

void runMove() {
  
}

void runServo() {

//  servoAngle = myservo.read();
//  Serial.print("Found angle ");
//  Serial.println(servoAngle);
  
  servoAngle += servoDirection;
  if ((servoAngle <= rightAngle) || (servoAngle >= leftAngle)) {
      servoDirection = 0 - servoDirection; // Change direction from + to - or - to +
  }

  servoAngle = max(min(leftAngle, servoAngle), rightAngle); // Clamp it to between left and right angles
  angleBucket = (int)servoAngle/(viewAngle/(viewPorts-1));
  
//  Serial.print("Running on: ");
//  Serial.print(millis());
  Serial.print(" with direction ");
  Serial.print(servoDirection);
  Serial.print(" and angle ");
  Serial.print(servoAngle);
  Serial.print(" bucket ");
  Serial.print(angleBucket);
  Serial.println();

  Serial.print("Ping: ");
  Serial.print(sonar.ping_in()); // Send ping, get distance and print result (0 = outside set distance range)
  Serial.println(" inches");
  
  myservo.write(servoAngle);
}

void setup() { 
  myservo.attach(3);  // attach servo on pin 3 to servo object
  Serial.begin(9600);     
  pinMode(Echo, INPUT);    
  pinMode(Trig, OUTPUT);  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  stop();

  // Configure threads
  sonarThread.onRun(runServo);
  sonarThread.setInterval(8);
  moveThread.onRun(runMove);
  moveThread.setInterval(100);
  
  control.add(&sonarThread);
  control.add(&moveThread);
} 

void loop() {
  Serial.println("Main loop");
  delay(loopTime);
  control.run();
}

