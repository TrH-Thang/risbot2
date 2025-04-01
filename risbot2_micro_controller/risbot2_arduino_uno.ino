#include <Arduino.h>
#include <AccelStepper.h>

// Định nghĩa chân kết nối với TB6600 (chỉnh lại cho Arduino Uno)
#define dir_pin_2        7
#define step_pin_2       6
#define dir_pin_1        5
#define step_pin_1       4
#define enable_1         8
#define enable_2         9

#define motorInterfaceType 1

// Khai báo động cơ bước
AccelStepper motor_right_step(motorInterfaceType, step_pin_1, dir_pin_1);
AccelStepper motor_left_step(motorInterfaceType, step_pin_2, dir_pin_2);

int step_right = 0;
int step_left = 0;

void setup() {
    Serial.begin(115200);

    // Cấu hình chân điều khiển driver
    pinMode(enable_1, OUTPUT);
    pinMode(enable_2, OUTPUT);
    digitalWrite(enable_1, LOW);  // Bật driver
    digitalWrite(enable_2, LOW);  // Bật driver

    // Cấu hình động cơ
    motor_right_step.setMaxSpeed(1000);
    motor_right_step.setAcceleration(500);
    motor_left_step.setMaxSpeed(1000);
    motor_left_step.setAcceleration(500);
}

void loop() {
    if (Serial.available()) {
        String received = Serial.readStringUntil('\n');
        sscanf(received.c_str(), "%d,%d", &step_right, &step_left);
        moveMotors();
        sendFeedback();
    }

    motor_right_step.run();
    motor_left_step.run();
}

void moveMotors() {
    motor_right_step.moveTo(motor_right_step.currentPosition() + step_right);
    motor_left_step.moveTo(motor_left_step.currentPosition() + step_left);
}

void sendFeedback() {
    Serial.print(motor_right_step.currentPosition());
    Serial.print(",");
    Serial.println(motor_left_step.currentPosition());
}
