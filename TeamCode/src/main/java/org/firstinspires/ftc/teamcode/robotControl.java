package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class robotControl {
    // Define hardware variables here

    public BNO055IMU imu;

    // Motors (DcMotor)

    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightFront;
    public DcMotor rightRear;

    public DcMotor wobbleArm;

    public DcMotor intake;

    public DcMotor shooter;

 //   public DcMotor rightWheel;
//    public DcMotor leftWheel;

//    public DcMotor rack;

    // Servos (Servo)

    public Servo wobbleClaw;
    public Servo flicker;
    public Servo lock;

    // Other
    public ElapsedTime timer = new ElapsedTime();

    private HardwareMap hardwareMap = null;

    public void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // Motors: variable = hardwareMap.dcMotor.get("name");
        leftFront = hardwareMap.dcMotor.get("0leftFront");
        leftRear = hardwareMap.dcMotor.get("1leftRear");
        rightFront = hardwareMap.dcMotor.get("2rightFront");
        rightRear = hardwareMap.dcMotor.get("3rightRear");

        wobbleArm = hardwareMap.dcMotor.get("wobbleArm");

        intake = hardwareMap.dcMotor.get("intake");

        shooter = hardwareMap.dcMotor.get("shooter");

  //      rightWheel = hardwareMap.dcMotor.get("rightWheel");
//        leftWheel = hardwareMap.dcMotor.get("leftWheel");

//        rack = hardwareMap.dcMotor.get("rack");


        // Direction: variable.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Zero Power: variable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

   //     rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        rack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos: variable = hardwareMap.servo.get("name");
        wobbleClaw = hardwareMap.servo.get("wobbleClaw");
        flicker = hardwareMap.servo.get("flicker");
        lock = hardwareMap.servo.get("lock");
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

    public static int detectRings(TFObjectDetector tfod, MultipleTelemetry telemetry) {
        int result = -1;

        int i = 0;
        while(true) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals("Quad") && updatedRecognitions.size() >= 1) {
                            telemetry.addData("Detected", "4");
                            if(i >= 30) {
                                result = 4;
                            }
                        } else if (recognition.getLabel().equals("Single") && updatedRecognitions.size() == 1) {
                            telemetry.addData("Detected", "1");
                            if(i >= 30) {
                                result = 1;
                            }
                        }
                    }
                    if (updatedRecognitions.size() == 0) {
                        telemetry.addData("Detected", "none");
                        if(i >= 30) {
                            result = 0;
                        }
                    }
                    i++;
                    telemetry.addData("i", i);
                    telemetry.addData("result", result);
                    telemetry.update();

                    if (result != -1) {
                        break;
                    }
                }
            }
        }

        return result;
    }

    public static int detectRingsConfidence(TFObjectDetector tfod, MultipleTelemetry telemetry) {
        int result = -1;

        int i = 0;
        while(true) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData("Confidence", recognition.getConfidence());
                        if (recognition.getLabel().equals("Quad") && updatedRecognitions.size() >= 1) {
                            telemetry.addData("Detected", "4");
                            if(i >= 15 || recognition.getConfidence() >= 0.85) {
                                result = 4;
                            }
                        } else if (recognition.getLabel().equals("Single") && updatedRecognitions.size() == 1) {
                            telemetry.addData("Detected", "1");
                            if(i >= 15 || recognition.getConfidence() >= 0.85) {
                                result = 1;
                            }
                        }
                    }
                    if (updatedRecognitions.size() == 0) {
                        telemetry.addData("Detected", "none");
                        if(i >= 15) {
                            result = 0;
                        }
                    }
                    i++;
                    telemetry.addData("i", i);
                    telemetry.addData("result", result);
                    telemetry.update();

                    if (result != -1) {
                        break;
                    }
                }
            }
        }

        return result;
        }
    }
