// Vacuum Robot with 3 Ultrasonic Sensors and L293D Motor Driver
// This code allows a robot to navigate autonomously using sensor input

// Ultrasonic Sensor Pins
// Sensor 1 - Front
const int trigPin1 = 7;
const int echoPin1 = 12;
// Sensor 2 - Left
const int trigPin2 = 11;
const int echoPin2 = 10;
// Sensor 3 - Right
const int trigPin3 = 13;
const int echoPin3 = 8;

// L293D Motor Driver Pins
// Motor A - Left motor
const int enableA = 1;  // Enable pin for motor A
const int in1 = 3;      // Motor A direction control 1
const int in2 = 2;     // Motor A direction control 2
// Motor B - Right motor
const int enableB = 9; // Enable pin for motor B
const int in3 = 5;     // Motor B direction control 1
const int in4 = 4;     // Motor B direction control 2

// Vacuum Motor Pin
const int vacuumMotor = A0;

// Constants
const int minDistance = 40;     // Minimum safe distance in cm
const int turnDistance = 50;    // Distance to start turning in cm

// Variables
long duration;
int distanceFront, distanceLeft, distanceRight;

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  
  // Configure Ultrasonic Sensor pins
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  
  // Configure L293D Motor Driver pins
  pinMode(enableA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enableB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Configure vacuum motor pin
  pinMode(vacuumMotor, OUTPUT);
  
  // Initialize motors - initially stopped
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  // Enable both motors - always on when needed
  analogWrite(enableA, 200);
  analogWrite(enableB, 200);
  
  // Turn on vacuum motor
  //digitalWrite(vacuumMotor, HIGH);
  
  // Short delay before starting
  delay(2000);
   bool stopper=true;
    while (stopper==true){
  
  // Read distances from all sensors
  distanceFront = readDistance(trigPin1, echoPin1);
  distanceLeft = readDistance(trigPin2, echoPin2);
  distanceRight = readDistance(trigPin3, echoPin3);
  
  // Print sensor readings for debugging
  Serial.print("Front: ");
  Serial.print(distanceFront);
  Serial.print(" cm, Left: ");
  Serial.print(distanceLeft);
  Serial.print(" cm, Right: ");
  Serial.print(distanceRight);
  Serial.println(" cm");
  
  // Navigation logic
  if (distanceFront > turnDistance) {
    // Clear path ahead - move forward
    moveForward();
  }
    if (distanceLeft < turnDistance) {
    // Clear path ahead - move forward
     clearLeft();
     }
     else if (distanceRight < turnDistance){
      clearRight();
      }
   else if (distanceFront <= minDistance) {
    // Too close to obstacle - back up and choose new direction
    moveBackward();
   
    delay(2000);
 
    
    // Decide which way to turn based on side sensor readings
    if (distanceRight > distanceLeft )  {
      turnRight();
    } else {
      turnLeft();
    }
   }
    delay(800); // Turn for a set amount of time
   if (distanceFront <= turnDistance) {
      if  (distanceRight<=10 && distanceLeft<=10) {
    // Too close to obstacle - back up and choose new direction
      stopMotors();
      break;
      }
    // Approaching obstacle - start turning
    else if (distanceRight > distanceLeft) {
      turnRight();
    } else {
      turnLeft();
    }
    delay(500); // Turn for a shorter time
  }
  
  // Small delay between readings
  delay(100);
}
}

void loop() {
}

// Function to read distance from ultrasonic sensor
int readDistance(int trigPin, int echoPin) {
  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(1);
  
  // Send a 10µs pulse to trigger
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echo pin - pulse duration in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate distance in centimeters
  // Speed of sound = 343m/s = 34300cm/s
  // Distance = (Time × Speed) / 2
  // Distance = (duration × 0.0343) / 2
  return duration * 0.0343 / 2;
}

// Motor control functions
void moveForward() {
  Serial.println("Moving Forward");
  
  // Motor A - Left motor
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  // Motor B - Right motor
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void moveBackward() {
  Serial.println("Moving Backward");
  
  // Motor A - Left motor
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  // Motor B - Right motor
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnLeft() {
  Serial.println("Turning Left");
  
  // Motor A - Left motor (reverse)
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  // Motor B - Right motor (forward)
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turnRight() {
  Serial.println("Turning Right");
  
  // Motor A - Left motor (forward)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  // Motor B - Right motor (reverse)
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void clearRight() {
  Serial.println("Clearing from right");
  //First turn to left, move forward, then right
  //Motor A - Left Motor (stop)
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  //Motor B - Right Motor (forward)
  digitalWrite(in3, LOW);
  digitalWrite(in4,HIGH);

  

  //Motor A - Left Motor (forward)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  //Motor B - Right Motor (stop)
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  }

void clearLeft() {
  Serial.println("Clearing from left");
  //First turn to right, move forward, then left
  //Motor A - Left Motor (forward)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  //Motor B - Right Motor (stop)
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  

  //Motor A - Left Motor (stop)
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  //Motor B - Right Motor (forward)
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  }

void stopMotors() {
  Serial.println("Stopping");
  
  // Set both motors to stop
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
