/*
 * Dijkstra's Algorithm for Robot Path Planning with Obstacle Avoidance
 * 
 * This code implements Dijkstra's algorithm for a robot to find the shortest path
 * while avoiding predefined obstacles. It uses:
 * - 4 DC motors with Cytron Motor Driver
 * - Optical encoders for distance measurement
 * - MPU6050 for orientation
 * 
 * The grid is defined as a 10x10 space where:
 * - (0,0) is the starting position
 * - (9,9) is the goal position
 * - Obstacles are predefined in the code
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "CytronMotorDriver.h"
#include "TimerOne.h"

// Define the grid size
#define GRID_SIZE 10
#define OBSTACLE_VALUE 99    // Value to represent obstacles
#define UNEXPLORED 255       // Value for unexplored cells

// Define motor pins
#define MOTOR_FRONT_LEFT_PWM 5
#define MOTOR_FRONT_LEFT_DIR 4
#define MOTOR_FRONT_RIGHT_PWM 11
#define MOTOR_FRONT_RIGHT_DIR 3
#define MOTOR_BACK_LEFT_PWM 6
#define MOTOR_BACK_LEFT_DIR 7
#define MOTOR_BACK_RIGHT_PWM 9
#define MOTOR_BACK_RIGHT_DIR 10

// Define encoder pins
#define ENCODER_FRONT 2  // Front encoder - Interrupt Pin 0
#define ENCODER_BACK 3   // Back encoder - Interrupt Pin 1

// Configure the motor drivers
CytronMD motorFL(PWM_DIR, MOTOR_FRONT_LEFT_PWM, MOTOR_FRONT_LEFT_DIR);
CytronMD motorFR(PWM_DIR, MOTOR_FRONT_RIGHT_PWM, MOTOR_FRONT_RIGHT_DIR);
CytronMD motorBL(PWM_DIR, MOTOR_BACK_LEFT_PWM, MOTOR_BACK_LEFT_DIR);
CytronMD motorBR(PWM_DIR, MOTOR_BACK_RIGHT_PWM, MOTOR_BACK_RIGHT_DIR);

// MPU6050 sensor
Adafruit_MPU6050 mpu;

// Encoder variables
volatile unsigned int counter1 = 0;  // Front encoder counter
volatile unsigned int counter2 = 0;  // Back encoder counter
float diskslots = 20;  // Number of slots in encoder disk
const float WHEEL_DIAMETER = 0.066;  // Wheel diameter in meters
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14;
const float ROBOT_WIDTH = 0.2;       // Distance between wheels in meters

// Define struct for distance measurement
struct DistanceResult {
    float distance1;
    float distance2;
};

// Robot position and orientation
int currentX = 0;
int currentY = 0;
float currentOrientation = 0; // in degrees, 0 = facing east

// Grid representation for Dijkstra's algorithm
byte grid[GRID_SIZE][GRID_SIZE];
byte visited[GRID_SIZE][GRID_SIZE];
byte previousX[GRID_SIZE][GRID_SIZE];
byte previousY[GRID_SIZE][GRID_SIZE];

// Define obstacle positions (x, y coordinates)
const byte NUM_OBSTACLES = 5;
byte obstacleX[NUM_OBSTACLES] = {2, 3, 5, 7, 8};
byte obstacleY[NUM_OBSTACLES] = {2, 4, 5, 3, 7};

// Path array to store the calculated path
#define MAX_PATH_LENGTH 50
byte pathX[MAX_PATH_LENGTH];
byte pathY[MAX_PATH_LENGTH];
byte pathLength = 0;
byte currentPathIndex = 0;

// Function declarations
void initializeGrid();
void runDijkstraAlgorithm();
void constructPath();
void moveRobot();
void goForward();
void turnLeft();
void turnRight();
void stopAllMotors();
void readMPU();

// Interrupt Service Routines for encoders
void ISR_count1() { counter1++; }  // Front encoder ISR
void ISR_count2() { counter2++; }  // Back encoder ISR

// Timer interrupt function
void ISR_timerone() {
  Timer1.detachInterrupt();
  Timer1.attachInterrupt(ISR_timerone);
}

// Get distance traveled function
DistanceResult getDistanceTravel() {
  Timer1.detachInterrupt();  // Stop the timer
  
  float rotation1 = (counter1 / diskslots) * 60.00;  // calculate RPM for Front Motor
  float distance1 = rotation1 * WHEEL_CIRCUMFERENCE * 60;
  counter1 = 0;  //  reset counter to zero
  
  float rotation2 = (counter2 / diskslots) * 60.00;  // calculate RPM for Back Motor
  float distance2 = rotation2 * WHEEL_CIRCUMFERENCE * 60;
  counter2 = 0;  //  reset counter to zero
  
  Timer1.attachInterrupt(ISR_timerone);  // Enable the timer
  
  DistanceResult result = {distance1, distance2};
  return result;
}

void setup() {
  Serial.begin(115200);
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 initialized!");
  
  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Setup encoder pins and interrupts
  pinMode(ENCODER_FRONT, INPUT_PULLUP);
  pinMode(ENCODER_BACK, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_FRONT), ISR_count1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_BACK), ISR_count2, RISING);
  
  // Initialize timer
  Timer1.initialize(1000000); // set timer for 1 second
  Timer1.attachInterrupt(ISR_timerone);
  
  // Initialize the grid with obstacles and run Dijkstra's algorithm
  initializeGrid();
  runDijkstraAlgorithm();
  constructPath();
  
  // Print the calculated path
  Serial.println("Calculated path:");
  for (int i = 0; i < pathLength; i++) {
    Serial.print("(");
    Serial.print(pathX[i]);
    Serial.print(",");
    Serial.print(pathY[i]);
    Serial.print(") -> ");
  }
  Serial.println("Goal");
  
  delay(2000);  // Short delay before starting to move
}

void loop() {
  // Read the MPU6050 for orientation
  readMPU();
  
  // Check if we've reached the goal
  if (currentX == GRID_SIZE-1 && currentY == GRID_SIZE-1) {
    Serial.println("Goal reached!");
    stopAllMotors();
    while(1) {
      delay(1000); // Stop and do nothing
    }
  }
  
  // Navigate to the next point in the path
  if (currentPathIndex < pathLength) {
    moveRobot();
  } else {
    stopAllMotors();
  }
  
  delay(100); // Small delay for stability
}

void initializeGrid() {
  // Initialize grid with unexplored values
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      grid[x][y] = UNEXPLORED;
      visited[x][y] = false;
      previousX[x][y] = 255; // Invalid value to indicate no predecessor
      previousY[x][y] = 255;
    }
  }
  
  // Place obstacles in the grid
  for (int i = 0; i < NUM_OBSTACLES; i++) {
    grid[obstacleX[i]][obstacleY[i]] = OBSTACLE_VALUE;
    Serial.print("Obstacle at (");
    Serial.print(obstacleX[i]);
    Serial.print(",");
    Serial.print(obstacleY[i]);
    Serial.println(")");
  }
  
  // Start position has distance 0
  grid[0][0] = 0;
}

void runDijkstraAlgorithm() {
  Serial.println("Running Dijkstra's Algorithm...");
  
  int x, y, distance;
  int minDistance, minX, minY;
  
  // Run until we've visited the goal position
  while (!visited[GRID_SIZE-1][GRID_SIZE-1]) {
    // Find the unvisited node with minimum distance
    minDistance = UNEXPLORED;
    minX = -1;
    minY = -1;
    
    for (y = 0; y < GRID_SIZE; y++) {
      for (x = 0; x < GRID_SIZE; x++) {
        if (!visited[x][y] && grid[x][y] < minDistance) {
          minDistance = grid[x][y];
          minX = x;
          minY = y;
        }
      }
    }
    
    // If we can't find a path to the goal
    if (minX == -1) {
      Serial.println("No path to goal found!");
      return;
    }
    
    // Mark the current node as visited
    visited[minX][minY] = true;
    distance = grid[minX][minY];
    
    // Check all neighboring cells
    // Right neighbor
    if (minX < GRID_SIZE-1 && grid[minX+1][minY] != OBSTACLE_VALUE && !visited[minX+1][minY]) {
      if (grid[minX+1][minY] == UNEXPLORED || grid[minX+1][minY] > distance + 1) {
        grid[minX+1][minY] = distance + 1;
        previousX[minX+1][minY] = minX;
        previousY[minX+1][minY] = minY;
      }
    }
    
    // Left neighbor
    if (minX > 0 && grid[minX-1][minY] != OBSTACLE_VALUE && !visited[minX-1][minY]) {
      if (grid[minX-1][minY] == UNEXPLORED || grid[minX-1][minY] > distance + 1) {
        grid[minX-1][minY] = distance + 1;
        previousX[minX-1][minY] = minX;
        previousY[minX-1][minY] = minY;
      }
    }
    
    // Up neighbor
    if (minY < GRID_SIZE-1 && grid[minX][minY+1] != OBSTACLE_VALUE && !visited[minX][minY+1]) {
      if (grid[minX][minY+1] == UNEXPLORED || grid[minX][minY+1] > distance + 1) {
        grid[minX][minY+1] = distance + 1;
        previousX[minX][minY+1] = minX;
        previousY[minX][minY+1] = minY;
      }
    }
    
    // Down neighbor
    if (minY > 0 && grid[minX][minY-1] != OBSTACLE_VALUE && !visited[minX][minY-1]) {
      if (grid[minX][minY-1] == UNEXPLORED || grid[minX][minY-1] > distance + 1) {
        grid[minX][minY-1] = distance + 1;
        previousX[minX][minY-1] = minX;
        previousY[minX][minY-1] = minY;
      }
    }
  }
  
  Serial.println("Dijkstra's Algorithm completed!");
}

void constructPath() {
  Serial.println("Constructing path...");
  
  // Start from the goal
  int x = GRID_SIZE - 1;
  int y = GRID_SIZE - 1;
  
  // Temporary arrays to build path in reverse
  byte tempX[MAX_PATH_LENGTH];
  byte tempY[MAX_PATH_LENGTH];
  byte tempLength = 0;
  
  // Backtrack from goal to start
  while (x != 0 || y != 0) {
    tempX[tempLength] = x;
    tempY[tempLength] = y;
    tempLength++;
    
    // Get the predecessor
    byte px = previousX[x][y];
    byte py = previousY[x][y];
    
    if (px == 255 || py == 255) {
      Serial.println("Error: Path construction failed!");
      return;
    }
    
    x = px;
    y = py;
  }
  
  // Reverse the path to get start-to-goal
  pathLength = tempLength;
  for (int i = 0; i < pathLength; i++) {
    pathX[i] = tempX[pathLength - 1 - i];
    pathY[i] = tempY[pathLength - 1 - i];
  }
  
  Serial.print("Path constructed with ");
  Serial.print(pathLength);
  Serial.println(" steps");
}

void moveRobot() {
  // Get next target position
  int targetX = pathX[currentPathIndex];
  int targetY = pathY[currentPathIndex];
  
  Serial.print("Moving to (");
  Serial.print(targetX);
  Serial.print(",");
  Serial.print(targetY);
  Serial.println(")");
  
  // Calculate the desired orientation
  float desiredOrientation = 0;
  
  if (targetX > currentX) desiredOrientation = 0;      // East
  else if (targetX < currentX) desiredOrientation = 180; // West
  else if (targetY > currentY) desiredOrientation = 90;  // North
  else if (targetY < currentY) desiredOrientation = 270; // South
  
  // Adjust orientation
  float orientationDiff = desiredOrientation - currentOrientation;
  
  // Normalize the difference to -180 to 180
  while (orientationDiff > 180) orientationDiff -= 360;
  while (orientationDiff < -180) orientationDiff += 360;
  
  // Turn to face the target
  if (orientationDiff > 15) {
    turnRight();
    delay(abs(orientationDiff) * 5); // Simple proportional control
    stopAllMotors();
    currentOrientation = desiredOrientation;
  } else if (orientationDiff < -15) {
    turnLeft();
    delay(abs(orientationDiff) * 5); // Simple proportional control
    stopAllMotors();
    currentOrientation = desiredOrientation;
  } else {
    // We're facing the right direction, move forward
    goForward();
    
    // Track distance using encoders until we reach the next grid cell
    const float CELL_SIZE = 0.2; // Distance between grid cells in meters
    float distanceTraveled = 0;
    
    while (distanceTraveled < CELL_SIZE) {
      // Get distance traveled from encoders
      DistanceResult dr = getDistanceTravel();
      
      // Average the distances from both encoders for more accuracy
      float stepDistance = (dr.distance1 + dr.distance2) / 2.0;
      distanceTraveled += stepDistance;
      
      Serial.print("Distance traveled: ");
      Serial.print(distanceTraveled);
      Serial.println(" meters");
      
      delay(100); // Small delay between readings
    }
    
    stopAllMotors();
    
    // Update current position
    currentX = targetX;
    currentY = targetY;
    
    // Move to next waypoint
    currentPathIndex++;
  }
}

void goForward() {
  motorFL.setSpeed(200);
  motorFR.setSpeed(200);
  motorBL.setSpeed(200);
  motorBR.setSpeed(200);
}

void turnLeft() {
  motorFL.setSpeed(-150);
  motorFR.setSpeed(150);
  motorBL.setSpeed(-150);
  motorBR.setSpeed(150);
}

void turnRight() {
  motorFL.setSpeed(150);
  motorFR.setSpeed(-150);
  motorBL.setSpeed(150);
  motorBR.setSpeed(-150);
}

void stopAllMotors() {
  motorFL.setSpeed(0);
  motorFR.setSpeed(0);
  motorBL.setSpeed(0);
  motorBR.setSpeed(0);
}

void readMPU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Update orientation based on gyroscope data
  // This is a simplified approach - in reality you would use a complementary or Kalman filter
  float gyroZ = g.gyro.z;
  currentOrientation += gyroZ * 0.1; // Simple integration
  
  // Normalize orientation to 0-360
  while (currentOrientation < 0) currentOrientation += 360;
  while (currentOrientation >= 360) currentOrientation -= 360;
  
  Serial.print("Current orientation: ");
  Serial.print(currentOrientation);
  Serial.println(" degrees");
}