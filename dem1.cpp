#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <iostream>
#include <vector>
#include <cmath>

#define TIME_STEP 64
#define MAX_SPEED 6.28

using namespace webots;

enum State {
    SCAN_ENVIRONMENT,
    SEARCH_BALL,
    MOVE_TO_BALL,
    GRAB_BALL,
    FIND_WALL,
    MOVE_TO_WALL,
    PLACE_BALL,
    BACK_UP,
    STOP
};

class BallSortingRobot {
private:
    Robot *robot;
    Motor *leftMotor, *rightMotor;
    Camera *camera;
    State currentState;
    int closestBallX;
    bool scanComplete;
    
public:
    BallSortingRobot() {
        robot = new Robot();
        leftMotor = robot->getMotor("left wheel motor");
        rightMotor = robot->getMotor("right wheel motor");
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
        
        camera = robot->getCamera("camera");
        camera->enable(TIME_STEP);
        
        currentState = SCAN_ENVIRONMENT;
        closestBallX = -1;
        scanComplete = false;
    }
    
    void rotate() {
        leftMotor->setVelocity(MAX_SPEED * 0.3);
        rightMotor->setVelocity(-MAX_SPEED * 0.3);
        std::cout << "Rotating to scan the environment..." << std::endl;
    }
    
    void moveForward() {
        leftMotor->setVelocity(MAX_SPEED * 0.5);
        rightMotor->setVelocity(MAX_SPEED * 0.5);
        std::cout << "Moving forward towards the ball." << std::endl;
    }
    
    void stop() {
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
        std::cout << "Robot stopped." << std::endl;
    }
    
    int detectClosestBall() {
        const unsigned char *image = camera->getImage();
        int width = camera->getWidth();
        int height = camera->getHeight();
        int closestX = -1;
        int maxSize = 0;
        
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int r = Camera::imageGetRed(image, width, x, y);
                int g = Camera::imageGetGreen(image, width, x, y);
                int b = Camera::imageGetBlue(image, width, x, y);
                
                if (r > 200 && g < 100 && b < 100) {
                    int size = 1;
                    while (x + size < width && Camera::imageGetRed(image, width, x + size, y) > 200) {
                        size++;
                    }
                    
                    if (size > maxSize) {
                        maxSize = size;
                        closestX = x;
                    }
                }
            }
        }
        std::cout << "Detected closest ball at X: " << closestX << std::endl;
        return closestX;
    }
    
    void alignToBall() {
        int center = camera->getWidth() / 2;
        if (closestBallX < center - 5) {
            leftMotor->setVelocity(-MAX_SPEED * 0.2);
            rightMotor->setVelocity(MAX_SPEED * 0.2);
            std::cout << "Aligning left to center the ball." << std::endl;
        } else if (closestBallX > center + 5) {
            leftMotor->setVelocity(MAX_SPEED * 0.2);
            rightMotor->setVelocity(-MAX_SPEED * 0.2);
            std::cout << "Aligning right to center the ball." << std::endl;
        } else {
            std::cout << "Ball aligned, moving towards it." << std::endl;
            currentState = MOVE_TO_BALL;
        }
    }
    
    void run() {
        while (robot->step(TIME_STEP) != -1) {
            switch (currentState) {
                case SCAN_ENVIRONMENT:
                    if (!scanComplete) {
                        rotate();
                        closestBallX = detectClosestBall();
                        if (closestBallX != -1) {
                            stop();
                            scanComplete = true;
                            currentState = SEARCH_BALL;
                        }
                    }
                    break;
                
                case SEARCH_BALL:
                    alignToBall();
                    break;
                
                case MOVE_TO_BALL:
                    moveForward();
                    if (closestBallX == -1) {
                        stop();
                        std::cout << "Reached the ball, attempting to grab it." << std::endl;
                        currentState = GRAB_BALL;
                    }
                    break;
                
                case GRAB_BALL:
                    std::cout << "Simulated grabbing of the ball." << std::endl;
                    currentState = FIND_WALL;
                    break;
                
                case FIND_WALL:
                    std::cout << "Searching for the correct wall..." << std::endl;
                    currentState = MOVE_TO_WALL;
                    break;
                
                case MOVE_TO_WALL:
                    std::cout << "Moving towards the correct wall." << std::endl;
                    currentState = PLACE_BALL;
                    break;
                
                case PLACE_BALL:
                    std::cout << "Placing ball in marked area." << std::endl;
                    currentState = BACK_UP;
                    break;
                
                case BACK_UP:
                    std::cout << "Backing up slightly." << std::endl;
                    scanComplete = false;
                    currentState = SCAN_ENVIRONMENT;
                    break;
                
                case STOP:
                    stop();
                    std::cout << "Task complete, stopping robot." << std::endl;
                    return;
            }
        }
    }
};

int main() {
    BallSortingRobot robot;
    robot.run();
    return 0;
}
