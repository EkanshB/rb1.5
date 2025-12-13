#include <Alfredo_NoU3.h>
#include <PestoLink-Receive.h>

//If your robot has more than a drivetrain, add those actuators here
NoU_Motor frontLeftMotor(7);
NoU_Motor frontRightMotor(2);
NoU_Motor rearLeftMotor(8);
NoU_Motor rearRightMotor(1);

//This creates the drivetrain object, you shouldn't have to mess with this
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

//The gyrospoce sensor is by default precise, but not accurate. This is fixable by adjusting the angular scale factor.
//Tuning procedure:
//Rotate the robot in place exactly 5 times. Use the Serial printour to read the current gyro angle in Radians, we will call this "measured_angle".
//measured_angle should be nearly 31.416 which is 5*2*pi. Update measured_angle below to complete the tuning process.
float measured_angle = 31.416;
float angular_scale = (5.0*2.0*PI) / measured_angle;

void setup() {
  PestoLink.begin("RB1.5");
  Serial.begin(115200);

  NoU3.begin();

  NoU3.calibrateIMUs(); // this takes exactly one second. Do not move robot during calibration.

  frontLeftMotor.setInverted(true);
  rearLeftMotor.setInverted(true);
}

void loop() {
    static unsigned long lastPrintTime = 0;
    if (lastPrintTime +100 < millis()){
        Serial.printf("gyro yaw (radians): %.3f\r\n", NoU3.yaw * angular_scale );
        lastPrintTime = millis();
    }

    //This measures your batterys voltage and sends it to PestoLink
    float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);

    if (PestoLink.isConnected()) {
        float fieldPowerX = PestoLink.getAxis(0);
        float fieldPowerY = -1 * PestoLink.getAxis(1);
        float rotationPower = -1 * PestoLink.getAxis(2);

        // Get robot heading (in radians) from the gyro
        float heading = NoU3.yaw * angular_scale;

        //Rotate joystick vector to be robot-centric
        //float cosA = cos(heading);
        //float sinA = sin(heading);

        //float robotPowerX = fieldPowerX * cosA + fieldPowerY * sinA;
        //float robotPowerY = -fieldPowerX * sinA + fieldPowerY * cosA;

        float robotPowerX = fieldPowerX + fieldPowerY;
        float robotPowerY = -fieldPowerX + fieldPowerY;

        //set motor power
        drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower);

        NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
        drivetrain.holonomicDrive(0, 0, 0); // stop motors if connection is lost

        NoU3.setServiceLight(LIGHT_DISABLED);
    }
}