#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>

#define TIME_STEP 64
#define MAX_SPEED 6.28

using namespace webots;

// Structure to hold color thresholds
struct ColorThreshold {
    std::string name;
    int minR, maxR;
    int minG, maxG;
    int minB, maxB;
};

enum State {
    SCAN_FOR_BALLS,
    MOVE_TO_BALL,
    SCAN_FOR_WALL,
    MOVE_TO_WALL,
    BACK_UP
};

class ColorSortingRobot {
private:
    Robot *robot;
    Motor *leftMotor, *rightMotor;
    Camera *camera;
    DistanceSensor *distanceSensor;
    State currentState;
    int targetBallX;
    int targetWallX;
    std::string currentBallColor;
    std::vector<ColorThreshold> colorThresholds;
    bool scanningComplete;
    double startTime;

public:
    ColorSortingRobot() {
        robot = new Robot();
        leftMotor = robot->getMotor("left wheel motor");
        rightMotor = robot->getMotor("right wheel motor");
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
        
        camera = robot->getCamera("camera");
        camera->enable(TIME_STEP);
        
        distanceSensor = robot->getDistanceSensor("distance sensor");
        distanceSensor->enable(TIME_STEP);
        
        currentState = SCAN_FOR_BALLS;
        targetBallX = -1;
        targetWallX = -1;
        scanningComplete = false;
        startTime = robot->getTime();
        
        setupColorThresholds();
    }

    void setupColorThresholds() {
        colorThresholds = {
            {"red",   200, 255, 0, 50,  0, 50},
            {"green", 0, 50,  200, 255, 0, 50},
            {"pink",  255, 255, 150, 200, 200, 255},
            {"yellow", 200, 255, 200, 255, 0, 50}
        };
    }

    void rotate(double speed) {
        leftMotor->setVelocity(speed);
        rightMotor->setVelocity(-speed);
    }

    void moveForward(double speed) {
        leftMotor->setVelocity(speed);
        rightMotor->setVelocity(speed);
    }

    void stop() {
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
    }

    std::string detectColor(int r, int g, int b) {
        for (const auto& color : colorThresholds) {
            if (r >= color.minR && r <= color.maxR &&
                g >= color.minG && g <= color.maxG &&
                b >= color.minB && b <= color.maxB) {
                return color.name;
            }
        }
        return "unknown";
    }

    bool detectObject(std::string targetColor, bool isWall) {
        const unsigned char *image = camera->getImage();
        int width = camera->getWidth();
        int height = camera->getHeight();
        int objectX = -1;

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int r = Camera::imageGetRed(image, width, x, y);
                int g = Camera::imageGetGreen(image, width, x, y);
                int b = Camera::imageGetBlue(image, width, x, y);
                
                if (detectColor(r, g, b) == targetColor) {
                    objectX = x;
                    break;
                }
            }
        }

        if (isWall) {
            targetWallX = objectX;
        } else {
            targetBallX = objectX;
        }
        
        return objectX != -1;
    }

    void run() {
        while (robot->step(TIME_STEP) != -1) {
            switch (currentState) {
                case SCAN_FOR_BALLS:
                    if (!scanningComplete) {
                        rotate(MAX_SPEED * 0.3);
                        for (const auto& color : colorThresholds) {
                            if (detectObject(color.name, false)) {
                                currentBallColor = color.name;
                                stop();
                                std::cout << "Found " << currentBallColor << " ball!" << std::endl;
                                currentState = MOVE_TO_BALL;
                                scanningComplete = true;
                                break;
                            }
                        }
                        if (robot->getTime() - startTime > 5.0) {
                            scanningComplete = true;
                            stop();
                        }
                    } else {
                        scanningComplete = false;
                        startTime = robot->getTime();
                    }
                    break;

                case MOVE_TO_BALL:
                    if (targetBallX != -1) {
                        int center = camera->getWidth() / 2;
                        if (targetBallX < center - 5) {
                            rotate(-MAX_SPEED * 0.2);
                        } else if (targetBallX > center + 5) {
                            rotate(MAX_SPEED * 0.2);
                        } else {
                            moveForward(MAX_SPEED * 0.5);
                        }
                        if (distanceSensor->getValue() < 100) {
                            stop();
                            currentState = SCAN_FOR_WALL;
                        }
                    }
                    break;

                case SCAN_FOR_WALL:
                    rotate(MAX_SPEED * 0.3);
                    if (detectObject(currentBallColor, true)) {
                        stop();
                        std::cout << "Found " << currentBallColor << " wall!" << std::endl;
                        currentState = MOVE_TO_WALL;
                    }
                    break;

                case MOVE_TO_WALL:
                    if (targetWallX != -1) {
                        int center = camera->getWidth() / 2;
                        if (targetWallX < center - 5) {
                            rotate(-MAX_SPEED * 0.2);
                        } else if (targetWallX > center + 5) {
                            rotate(MAX_SPEED * 0.2);
                        } else {
                            moveForward(MAX_SPEED * 0.5);
                        }
                        if (distanceSensor->getValue() < 50) {
                            stop();
                            currentState = BACK_UP;
                        }
                    }
                    break;

                case BACK_UP:
                    moveForward(-MAX_SPEED * 0.3);
                    robot->step(2000);
                    stop();
                    currentState = SCAN_FOR_BALLS;
                    break;
            }
        }
    }
};

int main() {
    ColorSortingRobot robot;
    robot.run();
    return 0;
}
