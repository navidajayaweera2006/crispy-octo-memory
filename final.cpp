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
    
    // Adjust these thresholds based on your sensor readings
    const double WHITE_THRESHOLD = 5.0;   // White background
    const double GREEN_THRESHOLD = 7.0;   // Green background
    const double BASE_SPEED = MAX_SPEED * 0.3;  // Reduced base speed for stability
    const double TURN_SPEED = MAX_SPEED * 0.15; // Gentler turning
    
    static int lostLineTimer = 0;  // Timer for tracking how long line has been lost
    const int TIMEOUT = (2000 / TIME_STEP);  // 2 seconds converted to timesteps
    
    // Check if line is detected by either sensor
    bool lineDetected = (leftValue > GREEN_THRESHOLD || rightValue > GREEN_THRESHOLD);
    
    if (!lineDetected) {
        lostLineTimer++;
        if (lostLineTimer >= TIMEOUT) {
            // Stop the robot after 2 seconds of no line
            setMotorSpeeds(0, 0);
            std::cout << "Line lost for 2 seconds - stopping robot" << std::endl;
            return;
        }
    } else {
        lostLineTimer = 0;  // Reset timer when line is detected
    }
    
    // Calculate the difference between sensors for proportional control
    double difference = leftValue - rightValue;
    double turnRate = difference * 0.1;
    
    // Line following logic
    if (leftValue > GREEN_THRESHOLD && rightValue > GREEN_THRESHOLD) {
        if (std::abs(difference) > 0.5) {
            setMotorSpeeds(BASE_SPEED - turnRate, BASE_SPEED + turnRate);
        } else {
            setMotorSpeeds(BASE_SPEED, BASE_SPEED);
        }
        std::cout << "Both sensors on line - diff: " << difference << std::endl;
    }
    else if (leftValue > GREEN_THRESHOLD) {
        setMotorSpeeds(BASE_SPEED + TURN_SPEED, BASE_SPEED - TURN_SPEED);
        std::cout << "Left sensor on line - correcting right" << std::endl;
    }
    else if (rightValue > GREEN_THRESHOLD) {
        setMotorSpeeds(BASE_SPEED - TURN_SPEED, BASE_SPEED + TURN_SPEED);
        std::cout << "Right sensor on line - correcting left" << std::endl;
    }
    else {
        static bool lastTurnDirection = true;
        if (lastTurnDirection) {
            setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
        } else {
            setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
        }
        static int searchCount = 0;
        if (++searchCount > 50) {
            lastTurnDirection = !lastTurnDirection;
            searchCount = 0;
        }
        std::cout << "Searching for line - Time without line: " << (lostLineTimer * TIME_STEP / 1000.0) << "s" << std::endl;
    }
}

  void run() {
    bool greenLineFound = false;
    int searchAttempts = 0; // To count the number of 90-degree turns
    
    while (step(TIME_STEP) != -1) {
        if (!greenLineFound && searchAttempts < 4) {
            // Turn 90 degrees
            setMotorSpeeds(SEARCH_SPEED, -SEARCH_SPEED);
            step(TIME_STEP * 20); // Adjusted for 90-degree turn
            setMotorSpeeds(0, 0);
            step(TIME_STEP * 2); // Brief pause after turn
            
            // Search for green line
            if (findGreenLine()) {
                greenLineFound = true;
                std::cout << "Green line found! Moving onto the line..." << std::endl;
                // Move forward to get onto the line
                setMotorSpeeds(MAX_SPEED * 0.5, MAX_SPEED * 0.5);
                step(TIME_STEP * 10);
                std::cout << "Switching to ground sensors..." << std::endl;
                setMotorSpeeds(0, 0);
                step(TIME_STEP * 2);
            } else {
                searchAttempts++;
            }
        } else if (greenLineFound) {
            followLineWithGroundSensors();
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
