package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.robotControl;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;

@Autonomous
@Config
public class twoWobbleAutoTesting extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AQgDy6j/////AAABmXISaVgPmUMmugAGhFFaTDBJsE9lRd9NNkRieex8wD8vq5MTahYK2UeRXAODvLE6NsZ/89ckvXlkMgAgRqlP/se9OTW0er/VNUu0M/eCAGXWgs0ji6it3/MAVY5tC4T5h/Inzn1NZQDa0XYMzCE2z5ionyuw/Vwk6rcqOsVqpxOYsLGMvcE++a64J8WJBoQHR9zSnStw++AarPQgqL0z9bQFbpopm2bw09FyY47tp4YvKl1kyYmPt0zbToit+D+r+WWQF0/1k1ikMaUt4SOhTRXHz/ZrzRW1jtnJOkYiCJ1qsxH8dH+2sS3dtrX1hzIZ+mKu4o+x1jsYClXc0P9p5x7bR1HGOH/cnsKLiBBly2QW";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private Trajectory goToRings, zero, one, four, getWobbleZero, deliverWobbleZero,
            getWobbleOne, deliverWobbleOne, getWobbleFour, deliverWobbleFour, shootPosZero,
            endLineZero, shootPosOne, endLineOne, shootPosFour, endLineFour;

    public static int result = 0;

    private double elapsed = 0.0;
    private long start = 0L;

    public static double wobbleClawOpen = 1.0;
    public static double wobbleClawClose = 0.0;

    public static int wobbleArmDistance = 420;

    public static double shooterPower = -0.68;
    public static double flickOpen = 0.85;
    public static double flickClose = 0.62;

    public static Vector2d goToRingsV1         = new Vector2d(-23.0, -58.0);
    public static Vector2d zeroV1              = new Vector2d(9.0, -50.0);
    public static Vector2d oneV1               = new Vector2d(38.0, -43.0);
    public static Vector2d fourV1              = new Vector2d(19.0, -42.0);
    public static Vector2d fourV2              = new Vector2d(36.5, -55.0);
    public static Vector2d fourV3              = new Vector2d(60.0, -49.0);
    public static Vector2d getWobbleZeroV1     = new Vector2d(-47.0, -11.0);
    public static Vector2d deliverWobbleZeroV1 = new Vector2d(14.0, -46.0);
    public static Vector2d getWobbleOneV1      = new Vector2d(-48.0, -12.0);
    public static Vector2d deliverWobbleOneV1  = new Vector2d(35.0, -20.0);
    public static Vector2d oneLineV1           = new Vector2d(14.0, -20.0);
    public static Vector2d getWobbleFourV1     = new Vector2d(-50.0, -40.0);
    public static Vector2d deliverWobbleFourV1 = new Vector2d(12.0, -42.0);
    public static Vector2d deliverWobbleFourV2 = new Vector2d(33.5, -42.0);
    public static Vector2d deliverWobbleFourV3 = new Vector2d(60.5, -42.0);
    public static Vector2d fourLineV1          = new Vector2d(14.0, -17.0);
    public static Vector2d endLineV1 = new Vector2d(14.0, -39.0);

    public static Vector2d highGoalPos = new Vector2d(0.0, -39.0);
    public static Vector2d powerShotPos = new Vector2d(-10.0, -17.0);

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.wobbleClaw.setPosition(wobbleClawClose);

        initVuforia();

        if(dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }
        else {
            telemetry = new MultipleTelemetry(telemetry);
        }

        initTfod();

        if(tfod != null) {
            tfod.activate();
        }

        Pose2d startPose = new Pose2d(-63.0, -50.0, Math.toRadians(0.0));

        drive.setPoseEstimate(startPose);
        goToRings = drive.trajectoryBuilder(startPose).lineToConstantHeading(goToRingsV1).build();
        // -23.0, -53.0

        zero = drive.trajectoryBuilder(goToRings.end()).lineToConstantHeading(zeroV1).build();

        one = drive.trajectoryBuilder(goToRings.end()).lineToConstantHeading(oneV1).build();

        four = drive.trajectoryBuilder(goToRings.end())
//                .lineToConstantHeading(fourV1)
//                .splineToConstantHeading(fourV2, Math.toRadians(0.0))
                .splineToConstantHeading(fourV3, Math.toRadians(0.0)).build();



        getWobbleZero = drive.trajectoryBuilder(new Pose2d(zero.end().getX(), zero.end().getY(), Math.toRadians(0.0)), Math.toRadians(90.0))
                .splineToConstantHeading(getWobbleZeroV1, Math.toRadians(180.0))
                .build();

        deliverWobbleZero = drive.trajectoryBuilder(getWobbleZero.end())
                .lineToConstantHeading(deliverWobbleZeroV1)
                .build();

        shootPosZero = drive.trajectoryBuilder(deliverWobbleZero.end())
                .lineToConstantHeading(highGoalPos)
                .build();

        endLineZero = drive.trajectoryBuilder(new Pose2d(shootPosZero.end().getX(), shootPosZero.end().getY(), Math.toRadians(170.0)))
                .lineToConstantHeading(endLineV1)
                .build();



        getWobbleOne = drive.trajectoryBuilder(goToRings.end())
                .addSpatialMarker(new Vector2d(37.0, -47.0), () -> {
                    moveWobbleArm(drive, 0.4, (wobbleArmDistance / 2) + 40);
                })
                .splineToConstantHeading(oneV1, Math.toRadians(90.0))
                .addSpatialMarker(new Vector2d(38.0, -27.0), () -> {
                    drive.wobbleClaw.setPosition(wobbleClawOpen);
                })
//                .addSpatialMarker(new Vector2d(0.0, -20.0), () -> {
//                    moveWobbleArm(drive, 0.4, (wobbleArmDistance / 2) - 40);
//                })
                .splineToConstantHeading(getWobbleOneV1, Math.toRadians(180.0))
                .build();

        deliverWobbleOne = drive.trajectoryBuilder(getWobbleOne.end())
                .lineToConstantHeading(deliverWobbleOneV1)
                .build();

        shootPosOne = drive.trajectoryBuilder(deliverWobbleOne.end(), Math.toRadians(160.0))
                .splineToConstantHeading(highGoalPos, Math.toRadians(200.0))
                .build();

        endLineOne = drive.trajectoryBuilder(new Pose2d(shootPosOne.end().getX(), shootPosOne.end().getY(), Math.toRadians(170.0)))
                .lineToConstantHeading(oneLineV1)
                .build();



        getWobbleFour = drive.trajectoryBuilder(four.end())
                .lineToConstantHeading(getWobbleFourV1)
                .build();

        shootPosFour = drive.trajectoryBuilder(four.end())
                .lineToConstantHeading(powerShotPos)
                .build();

        deliverWobbleFour = drive.trajectoryBuilder(getWobbleFour.end())
                .lineToConstantHeading(deliverWobbleFourV1)
                .splineToConstantHeading(deliverWobbleFourV2, Math.toRadians(0.0))
                .splineToConstantHeading(deliverWobbleFourV3, Math.toRadians(0.0))
                .build();

        endLineFour = drive.trajectoryBuilder(shootPosFour.end())
                .lineToConstantHeading(fourLineV1)
                .build();

        telemetry.addData(">", "Press Play to begin autonomous");
        telemetry.update();

        waitForStart();

        if(running()) {
            start = System.currentTimeMillis();

            drive.followTrajectory(goToRings);

            sleep(500);

            if(tfod != null) {
                result = robotControl.detectRingsConfidence(tfod, (MultipleTelemetry) telemetry);
            }

            telemetry.addData("Scan complete", result);
            telemetry.update();

            switch(result) {
                case 0:
                    drive.followTrajectory(zero);
                    moveWobbleArm(drive, 0.4, wobbleArmDistance);
                    sleep(100);
                    drive.wobbleClaw.setPosition(wobbleClawOpen);
                    sleep(200);
                    moveWobbleArm(drive, 0.4, -wobbleArmDistance);

                    drive.followTrajectory(getWobbleZero);
                    moveWobbleArm(drive, 0.4, wobbleArmDistance);
                    sleep(300);
                    drive.wobbleClaw.setPosition(wobbleClawClose);
                    sleep(500);
                    moveWobbleArm(drive, 0.4, -wobbleArmDistance);

                    drive.followTrajectory(deliverWobbleZero);
                    moveWobbleArm(drive, 0.4, wobbleArmDistance);
                    drive.wobbleClaw.setPosition(wobbleClawOpen);
                    sleep(100);
                    moveWobbleArm(drive, 0.4, -(wobbleArmDistance/2));

                    drive.followTrajectory(shootPosZero);
                    // shoot
                    // prime shooting wheel
                    drive.turn(Math.toRadians(170.0));
                    drive.shooter.setPower(shooterPower);
                    sleep(1200);

                    elapsed = (System.currentTimeMillis() - start) / 1000.0;
                    telemetry.addData("elapsed", elapsed);
                    telemetry.update();
                    if(elapsed >= 29.0) {
                        drive.shooter.setPower(0.0);
                        // put arm down plz
                    }
                    // move servo back and forth fast
                    for(int i = 0; i < 3; i++) {
                        drive.flicker.setPosition(flickOpen);
                        sleep(250);
                        if(elapsed >= 29.0) {
                            drive.shooter.setPower(0.0);
                            telemetry.addData("Put down arm now", "");
                            telemetry.update();
                            // put arm down plz
                            break;
                        }
                        drive.flicker.setPosition(flickClose);
                        sleep(250);
                    }
                    drive.shooter.setPower(0.0);

                    drive.followTrajectory(endLineZero);
                    break;
                case 1:
//                    moveWobbleArm(drive, 0.4, wobbleArmDistance);
                    drive.followTrajectory(getWobbleOne);

                    drive.wobbleClaw.setPosition(wobbleClawClose);
                    sleep(600);
                    moveWobbleArm(drive, 0.4, -wobbleArmDistance);

                    drive.followTrajectory(deliverWobbleOne);
                    moveWobbleArm(drive, 0.4, wobbleArmDistance);
                    drive.wobbleClaw.setPosition(wobbleClawOpen);
                    sleep(150);
                    moveWobbleArm(drive, 0.4, -wobbleArmDistance);

                    drive.followTrajectory(shootPosOne);
                    drive.turn(Math.toRadians(170.0));
                    drive.shooter.setPower(shooterPower);
                    sleep(1200);

                    elapsed = (System.currentTimeMillis() - start) / 1000.0;
                    telemetry.addData("elapsed", elapsed);
                    telemetry.update();
                    if(elapsed >= 29.0) {
                        drive.shooter.setPower(0.0);
                        // put arm down plz
                    }
                    // move servo back and forth fast
                    for(int i = 0; i < 3; i++) {
                        drive.flicker.setPosition(flickOpen);
                        sleep(250);
                        if(elapsed >= 29.0) {
                            drive.shooter.setPower(0.0);
                            telemetry.addData("Put down arm now", "");
                            telemetry.update();
                            // put arm down plz
                            break;
                        }
                        drive.flicker.setPosition(flickClose);
                        sleep(250);
                    }
                    drive.shooter.setPower(0.0);

                    drive.followTrajectory(endLineOne);
                    break;
                case 4:
                    shooterPower = -0.65;
                    drive.followTrajectory(four);
                    moveWobbleArm(drive, 0.4, wobbleArmDistance);
                    drive.wobbleClaw.setPosition(wobbleClawOpen);
                    sleep(150);
                    moveWobbleArm(drive, 0.4, -(wobbleArmDistance / 2));

                    drive.followTrajectory(shootPosFour);
                    drive.turn(Math.toRadians(170.0));
                    drive.shooter.setPower(shooterPower);
                    sleep(1200);

                    elapsed = (System.currentTimeMillis() - start) / 1000.0;
                    telemetry.addData("elapsed", elapsed);
                    telemetry.update();
                    if(elapsed >= 29.0) {
                        drive.shooter.setPower(0.0);
                        // put arm down plz
                    }
                    // move servo back and forth fast
                    for(int i = 0; i < 3; i++) {
                        drive.flicker.setPosition(flickOpen);
                        sleep(250);
                        if(elapsed >= 29.0) {
                            drive.shooter.setPower(0.0);
                            telemetry.addData("Put down arm now", "");
                            telemetry.update();
                            // put arm down plz
                            break;
                        }
                        drive.flicker.setPosition(flickClose);
                        drive.turn(Math.toRadians(-5.0));
                        sleep(250);
                    }
                    drive.shooter.setPower(0.0);

                    drive.followTrajectory(endLineFour);

                    // left powershot: 62in from bottom, 55in from right, 10deg
                    // 5deg increments
                    break;
                default:
                    drive.followTrajectory(returnToBaseline(goToRings.end(), drive));
                    break;
            }
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.

        if(dashboard != null) {
            dashboard.startCameraStream(vuforia, 0);
        }
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public boolean running() {
        return opModeIsActive() && !isStopRequested();
    }

    public Trajectory returnToBaseline(Pose2d endPoint, SampleMecanumDrive drive) {
        return drive.trajectoryBuilder(endPoint).lineToConstantHeading(new Vector2d(12.0, endPoint.getY())).build();
    }

    public void moveWobbleArm(SampleMecanumDrive drive, double power, int distance) {
        drive.wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.wobbleArm.setTargetPosition(distance);

        drive.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        drive.wobbleArm.setPower(power);

        while(drive.wobbleArm.isBusy()) {
            telemetry.addData("Motor position", drive.wobbleArm.getCurrentPosition());
            telemetry.update();
        }

        drive.wobbleArm.setPower(0.0);

        drive.wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
