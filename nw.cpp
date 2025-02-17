#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
#include <webots/DistanceSensor.hpp>
#include <cmath>
#include <iostream>

using namespace webots;

// Increased speed constants
#define MAX_SPEED 6.0        // Maximum speed
#define SEARCH_SPEED 5.0      // Speed while searching
#define APPROACH_SPEED 5.0    // Speed when approaching ball
#define WALL_SPEED 5.0        // Speed when going to wall
#define BACKUP_SPEED 6.0      // Speed when backing up

class EPuckController : public Robot {
public:
    EPuckController() : Robot() {
        timeStep = getBasicTimeStep();
        
        // Initialize devices
        gps = getGPS("gps");
        gps->enable(timeStep);
        
        compass = getCompass("compass");
        compass->enable(timeStep);
        
        camera = getCamera("camera");
        camera->enable(timeStep);
        
        // Initialize distance sensors
        for (int i = 0; i < 8; i++) {
            std::string name = "ps" + std::to_string(i);
            distanceSensors[i] = getDistanceSensor(name);
            distanceSensors[i]->enable(timeStep);
        }
        
        // Initialize motors
        leftMotor = getMotor("left wheel motor");
        rightMotor = getMotor("right wheel motor");
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        
        // Define wall positions
        wallPositions[0] = {0.5, 0.0};  // Red wall
        wallPositions[1] = {1.0, 0.5};  // Green wall
        wallPositions[2] = {0.5, 1.0};  // Yellow wall
        wallPositions[3] = {0.0, 0.5};  // Pink wall
        
        std::cout << "Robot initialized. Starting in FIND_BALL state." << std::endl;
    }

    void run() {
        while (step(timeStep) != -1) {
            // Read sensors
            const double *gpsValues = gps->getValues();
            double sensorValues[8];
            for (int i = 0; i < 8; i++) {
                sensorValues[i] = distanceSensors[i]->getValue();
            }

            // Print current position every few steps
            if (stepCounter % 50 == 0) {
                std::cout << "Current position: X=" << gpsValues[0] 
                         << " Z=" << gpsValues[2] << std::endl;
                std::cout << "Front sensors: Left=" << sensorValues[7] 
                         << " Right=" << sensorValues[0] << std::endl;
            }
            stepCounter++;

            switch (currentState) {
                case FIND_BALL:
                    findBall(sensorValues);
                    break;
                    
                case APPROACH_BALL:
                    approachBall(sensorValues);
                    break;
                    
                case CHECK_COLOR:
                    checkBallColor();
                    break;
                    
                case GOTO_WALL:
                    gotoWall(gpsValues);
                    break;
                    
                case BACKUP:
                    backupFromWall();
                    break;
            }
        }
    }

private:
    enum State {
        FIND_BALL,
        APPROACH_BALL,
        CHECK_COLOR,
        GOTO_WALL,
        BACKUP
    };

    int timeStep;
    GPS *gps;
    Compass *compass;
    Camera *camera;
    Motor *leftMotor, *rightMotor;
    DistanceSensor *distanceSensors[8];
    State currentState = FIND_BALL;
    int targetColor = -1;
    int stepCounter = 0;
    
    struct Position {
        double x, z;
    };
    Position wallPositions[4];

    void setSpeed(double left, double right) {
        leftMotor->setVelocity(left);
        rightMotor->setVelocity(right);
    }

    void findBall(const double *sensorValues) {
        bool ballFound = false;
        for (int i = 0; i < 8; i++) {
            if (sensorValues[i] > 80) {
                ballFound = true;
                std::cout << "Ball detected by sensor " << i 
                         << " (value: " << sensorValues[i] << ")" << std::endl;
                break;
            }
        }
        
        if (ballFound) {
            std::cout << "Ball found! Switching to APPROACH_BALL state" << std::endl;
            currentState = APPROACH_BALL;
        } else {
            // Rotate to search for ball
            setSpeed(-SEARCH_SPEED, SEARCH_SPEED);
        }
    }

    void approachBall(const double *sensorValues) {
        double leftSensor = sensorValues[7];
        double rightSensor = sensorValues[0];
        
        std::cout << "Approaching ball - Left sensor: " << leftSensor 
                 << " Right sensor: " << rightSensor << std::endl;
        
        if (leftSensor > 100 || rightSensor > 100) {
            std::cout << "Close to ball! Switching to CHECK_COLOR state" << std::endl;
            setSpeed(0.0, 0.0);
            currentState = CHECK_COLOR;
        } else if (leftSensor > rightSensor) {
            setSpeed(APPROACH_SPEED - 2.0, APPROACH_SPEED);
        } else if (rightSensor > leftSensor) {
            setSpeed(APPROACH_SPEED, APPROACH_SPEED - 2.0);
        } else {
            setSpeed(APPROACH_SPEED, APPROACH_SPEED);
        }
    }

    void checkBallColor() {
        const unsigned char *image = camera->getImage();
        int width = camera->getWidth();
        int height = camera->getHeight();
        int centerX = width / 2;
        int centerY = height / 2;
        
        int r = camera->imageGetRed(image, width, centerX, centerY);
        int g = camera->imageGetGreen(image, width, centerX, centerY);
        int b = camera->imageGetBlue(image, width, centerX, centerY);
        
        std::cout << "Color values - R:" << r << " G:" << g << " B:" << b << std::endl;
        
        if (r > 200 && g < 100 && b < 100) {
            targetColor = 0;
            std::cout << "Red ball detected!" << std::endl;
        } else if (g > 200 && r < 100 && b < 100) {
            targetColor = 1;
            std::cout << "Green ball detected!" << std::endl;
        } else if (r > 200 && g > 200 && b < 100) {
            targetColor = 2;
            std::cout << "Yellow ball detected!" << std::endl;
        } else if (r > 200 && b > 200 && g < 100) {
            targetColor = 3;
            std::cout << "Pink ball detected!" << std::endl;
        }
        
        if (targetColor != -1) {
            std::cout << "Moving to wall position: X=" << wallPositions[targetColor].x 
                     << " Z=" << wallPositions[targetColor].z << std::endl;
            currentState = GOTO_WALL;
        }
    }

    void gotoWall(const double *gpsValues) {
        if (targetColor == -1) return;
        
        double currentX = gpsValues[0];
        double currentZ = gpsValues[2];
        Position target = wallPositions[targetColor];
        
        double distance = sqrt(pow(target.x - currentX, 2) + 
                             pow(target.z - currentZ, 2));
        
        if (stepCounter % 50 == 0) {
            std::cout << "Distance to wall: " << distance << std::endl;
        }
        
        if (distance < 0.05) {
            std::cout << "Reached wall! Starting backup..." << std::endl;
            setSpeed(0.0, 0.0);
            currentState = BACKUP;
        } else {
            double angle = atan2(target.z - currentZ, target.x - currentX);
            double currentAngle = getAngle();
            double angleDiff = angle - currentAngle;
            
            while (angleDiff > M_PI) angleDiff -= 2 * M_PI;
            while (angleDiff < -M_PI) angleDiff += 2 * M_PI;
            
            if (fabs(angleDiff) > 0.1) {
                setSpeed(-WALL_SPEED * angleDiff, WALL_SPEED * angleDiff);
            } else {
                setSpeed(WALL_SPEED, WALL_SPEED);
            }
        }
    }

    double getAngle() {
        const double *north = compass->getValues();
        return atan2(north[0], north[2]);
    }

    void backupFromWall() {
        static int backupSteps = 0;
        if (backupSteps < 50) {
            setSpeed(-BACKUP_SPEED, -BACKUP_SPEED);
            backupSteps++;
        } else {
            std::cout << "Backup complete. Looking for next ball..." << std::endl;
            backupSteps = 0;
            setSpeed(0.0, 0.0);
            currentState = FIND_BALL;
            targetColor = -1;
        }
    }
};

int main() {
    EPuckController controller;
    controller.run();
    return 0;
}
