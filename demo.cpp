#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <algorithm>

using namespace webots;

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define SEARCH_SPEED 2.0

class LineFollowerRobot : public Robot {
public:
  LineFollowerRobot() {
    // Initialize camera for initial green line detection
    camera = getCamera("camera");
    camera->enable(TIME_STEP);
    
    // Initialize motors
    leftMotor = getMotor("left wheel motor");
    rightMotor = getMotor("right wheel motor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    
    // Initialize ground sensors
    groundSensorLeft = getDistanceSensor("gs0");
    groundSensorRight = getDistanceSensor("gs2");
    groundSensorLeft->enable(TIME_STEP);
    groundSensorRight->enable(TIME_STEP);
    
    // Initialize front distance sensors for cube detection
    ps[0] = getDistanceSensor("ps0");
    ps[7] = getDistanceSensor("ps7");
    ps[0]->enable(TIME_STEP);
    ps[7]->enable(TIME_STEP);
  }

  void setMotorSpeeds(double left, double right) {
    left = std::min(std::max(left, -MAX_SPEED), MAX_SPEED);
    right = std::min(std::max(right, -MAX_SPEED), MAX_SPEED);
    leftMotor->setVelocity(left);
    rightMotor->setVelocity(right);
  }

  bool isGreen(const unsigned char* image, int x, int y, int width) {
    int idx = (y * width + x) * 4;
    unsigned char r = image[idx];
    unsigned char g = image[idx + 1];
    unsigned char b = image[idx + 2];
    
    return (g > 100 && g > 1.5 * r && g > 1.5 * b);
  }

  bool findGreenLine() {
    const unsigned char* image = camera->getImage();
    int width = camera->getWidth();
    int height = camera->getHeight();
    
    int greenCount = 0;
    
    // Check bottom 5 rows of the image
    for (int y = height - 5; y < height; y++) {
      for (int x = 0; x < width; x++) {
        if (isGreen(image, x, y, width)) {
          greenCount++;
        }
      }
    }
    
    static int counter = 0;
    if (++counter % 10 == 0) {
      std::cout << "Green pixels found: " << greenCount << std::endl;
    }
    
    return greenCount > 5;
  }

  void followLineWithGroundSensors() {
    // Read ground sensor values
    double leftValue = groundSensorLeft->getValue();
    double rightValue = groundSensorRight->getValue();
    
    // Print sensor values for debugging
    std::cout << "Ground sensors - Left: " << leftValue << " Right: " << rightValue << std::endl;
    
    // Adjust these thresholds based on your testing
    const double LINE_THRESHOLD = 300;  // Adjust this value
    const double BASE_SPEED = MAX_SPEED * 0.3;
    const double TURN_SPEED = MAX_SPEED * 0.1;
    
    // Line following logic
    if (leftValue < LINE_THRESHOLD && rightValue < LINE_THRESHOLD) {
      // Both sensors on line - go straight
      setMotorSpeeds(BASE_SPEED, BASE_SPEED);
    } else if (leftValue < LINE_THRESHOLD && rightValue >= LINE_THRESHOLD) {
      // Left sensor on line, right sensor off line - turn left
      setMotorSpeeds(TURN_SPEED, BASE_SPEED);
    } else if (rightValue < LINE_THRESHOLD && leftValue >= LINE_THRESHOLD) {
      // Right sensor on line, left sensor off line - turn right
      setMotorSpeeds(BASE_SPEED, TURN_SPEED);
    } else {
      // Both sensors off line - perform a tight turn to search for the line
      setMotorSpeeds(-SEARCH_SPEED, SEARCH_SPEED);
    }
  }

  bool detectCube() {
    return ps[0]->getValue() > 80 || ps[7]->getValue() > 80;
  }

  void run() {
    bool greenLineFound = false;
    bool cubeReached = false;
    int searchAttempts = 0; // To count the number of 90-degree turns
    
    while (step(TIME_STEP) != -1) {
      if (!greenLineFound && searchAttempts < 4) {
        // Turn 90 degrees
        setMotorSpeeds(SEARCH_SPEED, -SEARCH_SPEED);
        step(TIME_STEP * 15); // Adjust this value based on your testing for a precise 90-degree turn
        setMotorSpeeds(0, 0);
        step(TIME_STEP * 2); // Brief pause after turn
        
        // Search for green line
        if (findGreenLine()) {
          greenLineFound = true;
          std::cout << "Green line found! Moving onto the line..." << std::endl;
          // Move forward to get onto the green line
          setMotorSpeeds(MAX_SPEED * 0.5, MAX_SPEED * 0.5);
          step(TIME_STEP * 10); // Adjust this value based on your testing
          std::cout << "Switching to ground sensors..." << std::endl;
          // Brief pause to stabilize
          setMotorSpeeds(0, 0);
          step(TIME_STEP * 2);
        } else {
          searchAttempts++;
        }
      } else if (greenLineFound && !cubeReached) {
        followLineWithGroundSensors();
        
        if (detectCube()) {
          cubeReached = true;
          std::cout << "Cube reached!" << std::endl;
        }
      } else if (cubeReached) {
        // Push cube to wall
        setMotorSpeeds(MAX_SPEED * 0.5, MAX_SPEED * 0.5);
      }
    }
  }

private:
  Camera *camera;
  Motor *leftMotor, *rightMotor;
  DistanceSensor *groundSensorLeft, *groundSensorRight;
  DistanceSensor *ps[8];
};

int main() {
  LineFollowerRobot *robot = new LineFollowerRobot();
  robot->run();
  delete robot;
  return 0;
}
