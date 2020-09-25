package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class robotControl {
    // Define hardware variables here

    // Motors (DcMotor)

    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightFront;
    public DcMotor rightRear;

    // Servos (Servo)


    // Other
    public ElapsedTime timer = new ElapsedTime();

    private HardwareMap hardwareMap = null;

    public void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;

        // Motors: variable = hardwareMap.dcMotor.get("name");

        // Direction: variable.setDirection(DcMotor.Direction.REVERSE);

        // Zero Power: variable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos: variable = hardwareMap.servo.get("name");
    }

    public void moveDriveMotors(double power) {
        moveDriveMotors(power, power);
    }

    public void moveDriveMotors(double leftPower, double rightPower) {
        moveDriveMotors(leftPower, leftPower, rightPower, rightPower);
    }

    public void moveDriveMotors(double leftFrontPower, double leftRearPower, double rightFrontPower, double rightRearPower) {
        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    public void halt() {
        moveDriveMotors(0.0);
    }

    public void encoderMode(DcMotor.RunMode mode) {
        rightFront.setMode(mode);
        rightRear.setMode(mode);
        leftFront.setMode(mode);
        leftRear.setMode(mode);
    }


}
