#include <MeMCore.h> 
MeRGBLed rgbLed(0,30);
MeBuzzer buzzer;  

// Define sensor pins
#define ULTRASONIC_PIN PORT_2
#define LINE_FOLLOWER_PIN PORT_1
const int irDetectorPin = A0;

//Set IR threshold
const int detectionThreshold = 100;

// Define desired distance to left wall (CM)
#define DESIRED_DISTANCE 10

// PID parameters
const float Kp = 5.0;   // Proportional gain
const float Ki = 0.0;    // Integral gain
const float Kd = 0.5;    // Derivative gain 

// Define motor ports
MeUltrasonicSensor ultrasonicSensor(ULTRASONIC_PIN);
MeLineFollower lineFollower(LINE_FOLLOWER_PIN);
MeDCMotor motorLeft(M1);
MeDCMotor motorRight(M2);

// Define motor speed variables
const int baseSpeed = 200;

// PID variables
float previousError = 0;
float integral = 0;
unsigned long previousTime = 0;
const unsigned long sampleTime = 50; // in milliseconds

// Define LDR brightness sensor
#define LDRbrightness A1

// Define a color structure to store RGB values and a label for the color
struct Color {
  int red;
  int green;
  int blue;
  String label;
};

// Dataset with 5 predefined colors
Color colorDataset[6] = {
  {695, 784, 739, "Red"},       // RGB values for Red
  {720, 838, 761, "Orange"},     // RGB values for Orange
  {604, 880, 785, "Green"},      // RGB values for Green
  {595, 873, 854, "Blue"},    // RGB values for Blue
  {767, 899, 867, "Pink"},    // RGB values for Pink
  {763, 927, 893, "White"}     // RGB values for Purple
};

// k value for k-NN)
const int k = 1;

// Function to calculate Euclidean distance between two colors
float calculateDistance(int r1, int g1, int b1, int r2, int g2, int b2) {
  return sqrt(pow(r1 - r2, 2) + pow(g1 - g2, 2) + pow(b1 - b2, 2));
}

// Function to classify the measured color using k-NN
String classifyColor(int red, int green, int blue) {
  float distances[6];

  // Calculate distance from the measured color to each color in the dataset
  for (int i = 0; i < 6; i++) {
    distances[i] = calculateDistance(red, green, blue, colorDataset[i].red, colorDataset[i].green, colorDataset[i].blue);
  }

  // Find the nearest neighbor (since k = 1, just find the minimum distance)
  int nearestIndex = 0;
  float minDistance = distances[0];
  for (int i = 1; i < 6; i++) {
    if (distances[i] < minDistance) {
      minDistance = distances[i];
      nearestIndex = i;
    }
  }

  // Return the label of the nearest color
  return colorDataset[nearestIndex].label;
}

// Function to play John Cena's intro music as a celebratory tune
void johnCenaIntro() {
  // Set BPM to 90 (each beat is 666.67 ms)
  const int beatDuration = 600;  // in milliseconds

  // Frequencies and durations inspired by John Cena's intro theme
  buzzer.tone(392, beatDuration); // G for 1 beat
  delay(int(beatDuration * 0.05 * 0.9));
  buzzer.tone(440, int(beatDuration * 0.5 * 0.9)); // A for half a beat
  delay(int(beatDuration * 0.005 * 0.9));  // Shortened delay
  buzzer.tone(349, int(beatDuration * 0.5 * 0.9)); // F for half a beat
  delay(int(beatDuration * 0.1 * 0.9));
  
  buzzer.noTone();
  delay(int(beatDuration * 0.1 * 0.9));

  buzzer.tone(392, int(beatDuration * 2 * 0.9)); // G for 2 beats
  delay(int(beatDuration * 0.3 * 0.9));

  buzzer.noTone();
  delay(int(beatDuration * 0.3 * 0.9));

  buzzer.tone(466, beatDuration); // Bb for 1 beat
  delay(int(beatDuration * 0.1 * 0.9));
  buzzer.tone(440, int(beatDuration * 0.5 * 0.9)); // A for half a beat
  delay(int(beatDuration * 0.005 * 0.9));  // Shortened delay
  buzzer.tone(349, int(beatDuration * 0.5 * 0.9)); // F for half a beat
  delay(int(beatDuration * 0.1 * 0.9));

  buzzer.noTone();
  delay(int(beatDuration * 0.1 * 0.9));

  buzzer.tone(392, int(beatDuration * 2 * 0.9)); // G for 2 beats
  delay(int(beatDuration * 0.3 * 0.9));

  buzzer.noTone();
}

//measureAndClassifyColor function
void measureAndClassifyColor() {
  int colour[3];
  colour[0] = measure_red();
  delay(500);
  off_lights();
  delay(500);
  colour[1] = measure_green();
  delay(500);
  off_lights();
  delay(500);
  colour[2] = measure_blue();
  delay(500);
  off_lights();
  delay(500);

  // Classify the measured color using k-NN
  String classifiedColor = classifyColor(colour[0], colour[1], colour[2]);

  //Color specific Logic
  if (classifiedColor == "White") {
    rgbLed.setColor(0, 255, 255, 255); // Set LED to white
    rgbLed.show();
    johnCenaIntro();
  }
  else if (classifiedColor == "Red") {
    rgbLed.setColor(0, 255, 0, 0);
    rgbLed.show();
    turnLeft();
  }
  else if (classifiedColor == "Green") {
    rgbLed.setColor(0, 0, 255, 0);
    rgbLed.show();
    turnRight();
  }
  else if (classifiedColor == "Blue") {
    rgbLed.setColor(0, 0, 0, 255);
    rgbLed.show();
    turnRight();
    delay(10); 
    moveForward();
    delay(1000); 
    turnRight();
  }
  else if (classifiedColor == "Pink") {
    rgbLed.setColor(0, 255, 105, 180);
    rgbLed.show();
    turnLeft();
    delay(10); 
    moveForward();
    delay(1000); 
    turnLeft();
  }
  else if (classifiedColor == "Orange") {
    rgbLed.setColor(0, 255, 165, 0);
    rgbLed.show();
    ReverseTurn();
  }
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  rgbLed.setpin(13);

  // Initialize pins for color measurement
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A1, INPUT);

  // Turn off lights initially
  off_lights();
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - previousTime >= sampleTime) {
    previousTime = currentTime;

    // Read the distance from the ultrasonic sensor
    int distance = ultrasonicSensor.distanceCm();

    // Read the line follower sensor status
    int lineSensorValue = lineFollower.readSensors();

    // Continuously monitor the line sensor until it detects a value of 0
    if (lineSensorValue == 0) {
      // Stop the robot when black line is detected
      stopMotors();

      // Turn on lights and perform color measurement
      measureAndClassifyColor();
    }
    else {
      //Ignore ultrasonic readings beyond 20 cm and drive straight
      if (distance > 20) {
        // Ignoring the distance value by moving forward without PID adjustments
        moveForward();
      }
      else {
        // Calculate PID
        float error = DESIRED_DISTANCE - distance;
        integral += error * (sampleTime / 1000.0);
        float derivative = (error - previousError) / (sampleTime / 1000.0);
        float output = Kp * error + Ki * integral + Kd * derivative;
        previousError = error;

        // Adjust motor speeds based on PID output
        int leftSpeed = -(baseSpeed + output);
        int rightSpeed = baseSpeed - output;

        // Constrain speeds to valid range
        leftSpeed = constrain(leftSpeed, -255, 255);
        rightSpeed = constrain(rightSpeed, -255, 255);

        motorLeft.run(leftSpeed);
        motorRight.run(rightSpeed);
      }
    }
  }

  //Check if LEDs are off before reading IR sensor
  if (digitalRead(A2) == HIGH && digitalRead(A3) == HIGH) {
    // Both LEDs are off, proceed with IR sensor reading
    int sensorValue = analogRead(irDetectorPin); // Read the analog value from IR detector
  
    if (sensorValue < detectionThreshold) { // If sensor value is above threshold, object is detected
      motorLeft.run(120);
      motorRight.run(180);
    }
  } 

  delay(10); // Small delay to prevent spamming the loop
}

// Function to stop both motors
void stopMotors() {
  motorLeft.run(0);
  motorRight.run(0);
}

// Function to turn left
void turnLeft() {
  motorLeft.run(baseSpeed);
  motorRight.run(baseSpeed);
  delay(430); 
  stopMotors();
}

// Function to turn right
void turnRight() {
  motorLeft.run(-baseSpeed);
  motorRight.run(-baseSpeed);
  delay(390);
  stopMotors();
}

//Function to reverse turn
void ReverseTurn() {
  turnRight();
  delay(8);
  turnRight();
  delay(8);
}

// Function to move forward
void moveForward() {
  motorLeft.run(-baseSpeed);
  motorRight.run(baseSpeed);
}

// Function to measure red light intensity
int measure_red() {
  digitalWrite(A2, LOW);  // Turn on red LED (A2 LOW, A3 LOW)
  digitalWrite(A3, LOW);
  delay(10);
  int red_reading = analogRead(LDRbrightness);
  return red_reading;
}

// Function to measure green light intensity
int measure_green() {
  digitalWrite(A2, HIGH);   // Turn on green LED (A2 HIGH, A3 LOW)
  digitalWrite(A3, LOW);
  delay(10);
  int green_reading = analogRead(LDRbrightness);
  return green_reading;
}

// Function to measure blue light intensity
int measure_blue() {
  digitalWrite(A2, LOW);   // Turn on blue LED (A2 LOW, A3 HIGH)
  digitalWrite(A3, HIGH);
  delay(10);
  int blue_reading = analogRead(LDRbrightness);
  return blue_reading;
}

// Function to turn off both LEDs
void off_lights() {
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
}