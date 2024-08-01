#include <SoftwareSerial.h>

// Motor control pins
#define IN1 2
#define IN2 3
#define ENA 5 // Motor 1 PWM pin

#define IN3 6
#define IN4 7
#define ENB 8 // Motor 2 PWM pin

// Bluetooth serial pins
#define BT_RX 9
#define BT_TX 10

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
        analogWrite(ENA, 255); // Adjusted PWM value for Uno

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, 255); // Adjusted PWM value for Uno
        break;
        
      case 'B': // Backward
        Serial.println("Motors moving backward at full speed.");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, 255); // Adjusted PWM value for Uno

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, 255); // Adjusted PWM value for Uno
        break;
        
      case 'L': // Left
        Serial.println("Motors turning left.");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, 255); // Adjusted PWM value for Uno

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, 255); // Adjusted PWM value for Uno
        break;
        
      case 'R': // Right
        Serial.println("Motors turning right.");
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 255); // Adjusted PWM value for Uno

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, 255); // Adjusted PWM value for Uno
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
