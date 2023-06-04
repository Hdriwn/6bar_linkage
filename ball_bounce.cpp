#include <iostream>
#include <cmath>
#include <fstream>
#include <string>

class Robot {
private:
    double d1, d2, d3, d4;
    
public:
    Robot(double d1, double d2, double d3, double d4) : d1(d1), d2(d2), d3(d3), d4(d4) {}
    
    bool checkValid(double theta1, double theta2, double theta3, double theta4, double a, double b) {
        bool isValidConfig = false;
        // Forward kinematics to calculate the position of the end effector
        double ex1 = d1 * cos(theta1) + d2 * cos(theta1 + theta2);
        double ex2 = d1 * cos(theta3) + d2 * cos(theta3 + theta4);
        if (ex1 < a && ex2 > a) {
            // Forward kinematics to calculate the position of the end effector
            double ey1 = d1 * sin(theta1) + d2 * sin(theta1 + theta2);
            double ey2 = d1 * sin(theta3) + d2 * sin(theta3 + theta4);
            if (ey1 < b && ey2 > b) {
                isValidConfig = true;
            }
        }
        return isValidConfig;
    }
    
    void calculateWorkspace(double a, double b) {
        bool isValidConfig = false;
        
        // Loop through theta1 and theta3
        for (double theta1 = 0; theta1 <= M_PI; theta1 += M_PI / 180.0) {
            for (double theta3 = 0; theta3 <= M_PI; theta3 += M_PI / 180.0) {
                for (double theta2 = 0; theta2 <= 2 * M_PI; theta2 += M_PI / 180.0) {
                    for (double theta4 = 0; theta4 <= 2 * M_PI; theta4 += M_PI / 180.0) {
                        bool validWorkspace = checkValid(theta1, theta2, theta3, theta4, a, b);
                        if (validWorkspace) {
                            isValidConfig = true;
                            break;
                        }
                    }
                }

            }
        }
        
        if (isValidConfig) {
            std::cout << "Valid config" << std::endl;
        } else {
            std::cout << "Invalid config" << std::endl;
        }
    }
    
    void move_to_angle(int motor, double theta) {
        // Move the motor to the specified angle
    }

    void actuate(double theta1, double theta2, double theta3, double theta4) {
        // Move the motors to the calculated angles
        move_to_angle(11, theta1);
        move_to_angle(12, theta2);
        move_to_angle(21, theta3);
        move_to_angle(22, theta4);
        std::cout << "Jerk motion completed!" << std::endl;
    }
double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}
double radiansToDegrees(double radians) {
    return radians * 180.0 / M_PI;
}
    void orientandjerk(double a, double b, double ball_in_range, double ball_theta) {
        // Solving for angles based on the position of ees
        // E1 = e1x, e1y and E2 = e2x, e2y
        double e1x = a * sin(ball_theta) - (d4 / 2) * sin(ball_theta); // d4*cos(90-ball_theta)
        double e1y = b * cos(ball_theta) - (d4 / 2) * cos(ball_theta); // d4*sin(90-ball_theta)
    
        double e2x = a * sin(ball_theta) + (d4 / 2) * sin(ball_theta);
        double e2y = b * cos(ball_theta) + (d4 / 2) * cos(ball_theta);
    
        // Solving for angles based on the position of end effector
        double theta1 = degreesToRadians(150); // keeping the angles for theta1 and theta3 fixed at 120 and 60 degrees
        double theta3 = degreesToRadians(60);
        // perform inverse kinematics
        /***
         * The following are the equations for coordinates of Platforms end points
            1. e1x = -d3/2 + d1cos(theta1) + d2 cos(theta1+theta2)
            2. e1x = d3/2  + d1cos(theta3) + d2 cos( theta2+theta4) - d3 sin(theta)
            3. e1y = d1sin(theta1)+ d2 sin(theta1+theta2)
            4. e1y = d1sin(theta3)+d2sin(theta2+theta4) - d4cos(theta)
            5. e2x = d3/2 + d1 cos(theta3)+ d2 cos(theta3+theta4)
            6. e2x= -d3/2 + d1cos(theta1) + d2 cos(theta1+theta2) + d4 sin(theta)
            7. e2y = d1 sin(theta3) + d2 sin(theta3 + theta4)
            8. e2y= d1 cos(theta1)+ d2 cos(theta1+theta2) + d4cos(theta)
        ****/
        double theta2 = radiansToDegrees(acos(((a - d4/2) * sin(ball_theta) - (-d3/2 + d1*cos(theta1))) / d2) - theta1); 
        double theta4 = radiansToDegrees(acos(((a + d4/2) * sin(ball_theta) - (d3/2 + d1*cos(theta3))) / d2) - theta3);
        
        // whether the position is valid
        bool validChecked = checkValid(theta1, theta2, theta3, theta4, a, b);
        if (ball_in_range && validChecked) {
            actuate(theta1, theta2, theta3, theta4);
            // Perform jerk motion
            actuate(theta1, theta2 - 5, theta3, theta4 - 5); // just a 5 degree angle to the jerk
            // again go the proper position
            actuate(theta1, theta2, theta3, theta4);
        } else {
            std::cout << "Unable to perform the action" << std::endl;
        }
    
        std::cout << "Theta1: " << theta1 << " degrees" << std::endl;
        std::cout << "Theta2: " << theta2 << " degrees" << std::endl;
        std::cout << "Theta3: " << theta3 << " degrees" << std::endl;
        std::cout << "Theta4: " << theta4 << " degrees" << std::endl;
    }
};
bool readConfigFile(const std::string& filename, double& d1, double& d2, double& d3, double& d4, double& a, double& b, double& ball_theta) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "Error opening config file: " << filename << std::endl;
        return false;
    }

    file >> d1 >> d2 >> d3 >> d4 >> a >> b >> ball_theta;

    file.close();
    return true;
}

int main() {
     std::string configFile = "config.txt";
     // write the config text according to this
    double d1, d2, d3, d4, a, b, ball_theta;

    if (readConfigFile(configFile, d1, d2, d3, d4, a, b, ball_theta)) {
        std::cout << "d1 = " << d1 << std::endl;
        std::cout << "d2 = " << d2 << std::endl;
        std::cout << "d3 = " << d3 << std::endl;
        std::cout << "d4 = " << d4 << std::endl;
        std::cout << "a = " << a << std::endl;
        std::cout << "b = " << b << std::endl;
        std::cout << "theta = " << ball_theta << std::endl;

    }
    ball_theta =  ball_theta* M_PI / 180.0;
    // robot with the above dimentsions
    Robot robot(d1, d2, d3, d4);

    // checking for valid workspace
    robot.calculateWorkspace(a, b);

    // bounce the ball
    robot.orientandjerk(a, b, true, ball_theta);

    return 0;
}
