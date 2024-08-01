#include <SoftwareSerial.h>

// Motor control pins
#define IN1 D1
#define IN2 D2
#define ENA D3 // Motor 1 PWM pin

#define IN3 D5
#define IN4 D6
#define ENB D7 // Motor 2 PWM pin

// Bluetooth serial pins
#define BT_RX D9
#define BT_TX D10

// Create a SoftwareSerial object for Bluetooth communication
SoftwareSerial BTSerial(BT_RX, BT_TX);

void setup() {
  // Start the hardware serial port for debugging
  Serial.begin(115200);
  Serial.println("Serial communication started.");

  // Start the Bluetooth serial port
  BTSerial.begin(9600);
  Serial.println("Bluetooth module ready.");

  // Set motor control pins as output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initially stop both motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0); // Stop Motor 1
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0); // Stop Motor 2

  Serial.println("Motors initialized and stopped.");
  Serial.println("Waiting for Bluetooth commands...");
}

void loop() {
  // Check if data is available on the Bluetooth serial port
  if (BTSerial.available()) {
    char command = BTSerial.read();
    
    // Process Bluetooth commands
    switch(command) {
      case 'F': // Forward
        Serial.println("Motors moving forward at full speed.");
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 1023); // Motor 1 full speed

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, 1023); // Motor 2 full speed
        break;
        
      case 'B': // Backward
        Serial.println("Motors moving backward at full speed.");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, 1023); // Motor 1 full speed

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, 1023); // Motor 2 full speed
        break;
        
      case 'L': // Left
        Serial.println("Motors turning left.");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, 1023); // Motor 1 full speed

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, 1023); // Motor 2 full speed
        break;
        
      case 'R': // Right
        Serial.println("Motors turning right.");
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 1023); // Motor 1 full speed

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, 1023); // Motor 2 full speed
        break;
        
      case 'S': // Stop
        Serial.println("Motors stopped.");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 0); // Stop Motor 1

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, 0); // Stop Motor 2
        break;
        
      default:
        Serial.println("Invalid command.");
        break;
    }
  }
  
  // Check for data from Serial Monitor (for debugging)
  if (Serial.available()) {
    char command = Serial.read();
    BTSerial.print(command);
  }

  // Delay to allow time for Bluetooth communication
  delay(100);
}