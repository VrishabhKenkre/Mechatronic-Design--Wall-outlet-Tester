#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Stepper.h>

// Define Stepper Motor Pins
#define IN1 7
#define IN2 6
#define IN3 5
#define IN4 4

// Steps per revolution for 28BYJ-48 stepper motor
#define STEPS_PER_REV 2048

// IMUSensor Class
class IMUSensor {
private:
    Adafruit_MPU6050 mpu;
    float last_yaw;

public:
    IMUSensor() : last_yaw(0.0) {}

    void start() {
        Serial.begin(115200);
        Wire.begin();
        
        Serial.println("Initializing MPU6050...");
        if (!mpu.begin()) {
            Serial.println("MPU6050 Connection Failed! Check wiring.");
            while (1);
        }
        Serial.println("MPU6050 Initialized Successfully!");

        // Configure IMU
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

        delay(2000); // Allow IMU to stabilize
    }

    float get_yaw_change() {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Convert gyro Z from rad/s to degrees/sec
        float current_yaw = last_yaw + (g.gyro.z * 57.2958) * 0.01;
        float yaw_change = current_yaw - last_yaw;

        Serial.print("Yaw: ");
        Serial.println(current_yaw);

        last_yaw = current_yaw;
        return yaw_change;
    }
};

// StepperMotor Class
class StepperMotor {
private:
    Stepper stepperMotor;
    IMUSensor &imu; // Reference to IMU object

public:
    StepperMotor(IMUSensor &imuRef) : stepperMotor(STEPS_PER_REV, IN1, IN3, IN2, IN4), imu(imuRef) {
        stepperMotor.setSpeed(10);  // Set motor speed (RPM)
    }

    void update() {
        float yaw_change = imu.get_yaw_change();
        int steps = (int)(yaw_change / (360.0 / STEPS_PER_REV));

        if (abs(steps) > 1) {
            stepperMotor.step(steps);  // Rotate stepper motor
        }
    }
};

// Create global objects
IMUSensor imu;
StepperMotor stepper(imu);

void setup() {
    imu.start();  // Start IMU
}

void loop() {
    stepper.update();  // Move motor based on IMU data
    delay(10);
}

