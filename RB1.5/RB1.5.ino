#include <Alfredo_NoU3.h>
#include <PestoLink-Receive.h>

NoU_Motor frontLeftMotor(8);
NoU_Motor frontRightMotor(7);
NoU_Motor rearLeftMotor(1);
NoU_Motor rearRightMotor(2);

NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

//Tuning procedure: 
//Rotate the robot in place 5 times. Use the Serial printout to read the current gyro angle in Radians, we will call this "measured_angle".
//measured_angle should be nearly 31.416 which is 5*2*pi. Update measured_angle below to complete the tuning process. 
float measured_angle = 31.416;
float angular_scale = (5.0*2.0*PI) / measured_angle;

void setup() {
    PestoLink.begin("RB1.5");
    Serial.begin(115200);

    NoU3.begin();
    
    NoU3.calibrateIMUs();

    frontLeftMotor.setInverted(true);
    rearLeftMotor.setInverted(true);    
}
    
unsigned long lastPrintTime = 0;

void loop() {

    if (lastPrintTime + 100 < millis()){
        Serial.printf("gyro yaw (radians): %.3f\r\n",  NoU3.yaw * angular_scale );
        lastPrintTime = millis();
    }

    float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);

    if (PestoLink.isConnected()) {
        float fieldPowerX = PestoLink.getAxis(0);
        float fieldPowerY = -PestoLink.getAxis(1);
        float rotationPower = -PestoLink.getAxis(2);

        // Get robot heading (in radians) from the gyro
        float heading = NoU3.yaw * angular_scale;

        float robotPowerX = fieldPowerX+ fieldPowerY;
        float robotPowerY = -fieldPowerX + fieldPowerY;

        //set motor power
        drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower);

        NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
        drivetrain.holonomicDrive(0, 0, 0); // stop motors if connection is lost

        NoU3.setServiceLight(LIGHT_DISABLED);
    }
    PestoLink.update();
}