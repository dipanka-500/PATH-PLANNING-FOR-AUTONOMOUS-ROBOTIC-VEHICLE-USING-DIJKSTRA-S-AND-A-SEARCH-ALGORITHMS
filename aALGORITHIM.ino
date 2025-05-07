/*
 * A* Algorithm for Robot Path Planning with Obstacle Avoidance
 * 
 * This code implements A* algorithm for a robot to find the shortest path
 * while avoiding predefined obstacles. It uses:
 * - 4 DC motors with Cytron Motor Driver
 * - 2 Optical encoders (front and back) for distance measurement
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
#define MAX_OPEN_LIST 50     // Maximum size for the open list

// Define motor pins
#define MOTOR_FRONT_LEFT_PWM 3
#define MOTOR_FRONT_LEFT_DIR 4
#define MOTOR_FRONT_RIGHT_PWM 5
#define MOTOR_FRONT_RIGHT_DIR 6
#define MOTOR_BACK_LEFT_PWM 7
#define MOTOR_BACK_LEFT_DIR 8
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

// A* algorithm data structures
typedef struct {
  int x, y;
  int g;       // Cost from start to this node
  int h;       // Heuristic cost (estimated distance to goal)
  int f;       // Total cost (g + h)
  int parentX, parentY;
} Node;

// Grid representation
bool obstacles[GRID_SIZE][GRID_SIZE];
Node grid[GRID_SIZE][GRID_SIZE];
bool closed[GRID_SIZE][GRID_SIZE];

// Open list for A* algorithm
Node openList[MAX_OPEN_LIST];
int openListCount = 0;

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
void initializeAStar();
void runAStarAlgorithm();
void constructPath();
void moveRobot();
void goForward();
void turnLeft();
void turnRight();
void stopAllMotors();
void readMPU();
int calculateHeuristic(int x, int y);
void addToOpenList(Node node);
int findLowestFInOpenList();
void removeFromOpenList(int index);

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
  
  // Initialize the grid with obstacles and run A* algorithm
  initializeAStar();
  runAStarAlgorithm();
  constructPath();
  
  // Print the calculated path
  Serial.println("Calculated path using A*:");
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

void initializeAStar() {
  Serial.println("Initializing A* algorithm...");
  
  // Initialize grid
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      obstacles[x][y] = false;
      closed[x][y] = false;
      
      grid[x][y].x = x;
      grid[x][y].y = y;
      grid[x][y].g = UNEXPLORED;
      grid[x][y].h = calculateHeuristic(x, y);
      grid[x][y].f = UNEXPLORED;
      grid[x][y].parentX = -1;
      grid[x][y].parentY = -1;
    }
  }
  
  // Place obstacles in the grid
  for (int i = 0; i < NUM_OBSTACLES; i++) {
    obstacles[obstacleX[i]][obstacleY[i]] = true;
    Serial.print("Obstacle at (");
    Serial.print(obstacleX[i]);
    Serial.print(",");
    Serial.print(obstacleY[i]);
    Serial.println(")");
  }
  
  // Set start node
  grid[0][0].g = 0;
  grid[0][0].f = grid[0][0].h;
  
  // Initialize open list
  openListCount = 0;
  addToOpenList(grid[0][0]);
}

// Calculate Manhattan distance heuristic
int calculateHeuristic(int x, int y) {
  // Manhattan distance to goal (9,9)
  return abs(GRID_SIZE - 1 - x) + abs(GRID_SIZE - 1 - y);
}

// Add a node to the open list
void addToOpenList(Node node) {
  if (openListCount >= MAX_OPEN_LIST) {
    Serial.println("Error: Open list is full");
    return;
  }
  
  openList[openListCount] = node;
  openListCount++;
}

// Find the node with lowest F value in open list
int findLowestFInOpenList() {
  if (openListCount == 0) return -1;
  
  int lowestIndex = 0;
  int lowestF = openList[0].f;
  
  for (int i = 1; i < openListCount; i++) {
    if (openList[i].f < lowestF) {
      lowestF = openList[i].f;
      lowestIndex = i;
    }
  }
  
  return lowestIndex;
}

// Remove a node from the open list
void removeFromOpenList(int index) {
  if (index < 0 || index >= openListCount) return;
  
  // Replace with the last element and decrease count
  openList[index] = openList[openListCount - 1];
  openListCount--;
}

void runAStarAlgorithm() {
  Serial.println("Running A* Algorithm...");
  
  // Algorithm runs until we've either found a path or determined none exists
  while (openListCount > 0) {
    // Find the node with lowest F value
    int currentIndex = findLowestFInOpenList();
    Node current = openList[currentIndex];
    
    // Remove it from open list
    removeFromOpenList(currentIndex);
    
    // Mark as closed
    closed[current.x][current.y] = true;
    
    // Check if we've reached the goal
    if (current.x == GRID_SIZE-1 && current.y == GRID_SIZE-1) {
      Serial.println("Path found!");
      return;
    }
    
    // Check all adjacent squares
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        // Skip diagonals and center (current cell)
        if (abs(dx) + abs(dy) != 1) continue;
        
        int nx = current.x + dx;
        int ny = current.y + dy;
        
        // Check bounds
        if (nx < 0 || nx >= GRID_SIZE || ny < 0 || ny >= GRID_SIZE) continue;
        
        // Skip obstacles and closed nodes
        if (obstacles[nx][ny] || closed[nx][ny]) continue;
        
        // Calculate new G value (cost from start)
        int newG = current.g + 1;
        
        // Check if this is a better path
        if (grid[nx][ny].g == UNEXPLORED || newG < grid[nx][ny].g) {
          // Update the grid node
          grid[nx][ny].g = newG;
          grid[nx][ny].f = newG + grid[nx][ny].h;
          grid[nx][ny].parentX = current.x;
          grid[nx][ny].parentY = current.y;
          
          // Add to open list if not already there
          bool inOpenList = false;
          for (int i = 0; i < openListCount; i++) {
            if (openList[i].x == nx && openList[i].y == ny) {
              inOpenList = true;
              openList[i] = grid[nx][ny]; // Update with new values
              break;
            }
          }
          
          if (!inOpenList) {
            addToOpenList(grid[nx][ny]);
          }
        }
      }
    }
  }
  
  Serial.println("No path found to goal!");
}

void constructPath() {
  Serial.println("Constructing path...");
  
  // Start from the goal
  int x = GRID_SIZE - 1;
  int y = GRID_SIZE - 1;
  
  // Check if a path exists
  if (grid[x][y].parentX == -1) {
    Serial.println("No valid path to construct!");
    pathLength = 0;
    return;
  }
  
  // Temporary arrays to build path in reverse
  byte tempX[MAX_PATH_LENGTH];
  byte tempY[MAX_PATH_LENGTH];
  byte tempLength = 0;
  
  // Backtrack from goal to start
  while (!(x == 0 && y == 0)) {
    tempX[tempLength] = x;
    tempY[tempLength] = y;
    tempLength++;
    
    // Get the parent cell
    int px = grid[x][y].parentX;
    int py = grid[x][y].parentY;
    
    // Sanity check
    if (px < 0 || py < 0 || tempLength >= MAX_PATH_LENGTH) {
      Serial.println("Error in path construction!");
      break;
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