#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#define RIGHT_MOTOR_ENABLE 10
#define LEFT_MOTOR_ENABLE 5
#define RIGHT_MOTOR_FORWARD 9
#define RIGHT_MOTOR_BACKWARD 8
#define LEFT_MOTOR_FORWARD 7
#define LEFT_MOTOR_BACKWARD 6

enum side {LEFT, RIGHT};

void initMotorController();
void enableMotors();
void enableRightMotor();
void enableLeftMotor();
void disableMotors();
void disableRightMotor();
void disableLeftMotor();
void setMoterSpeed(side motor_side, int speed);
void setMotorSpeeds();

#endif