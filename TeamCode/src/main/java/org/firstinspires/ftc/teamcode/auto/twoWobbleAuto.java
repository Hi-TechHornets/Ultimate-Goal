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
public class twoWobbleAuto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AQgDy6j/////AAABmXISaVgPmUMmugAGhFFaTDBJsE9lRd9NNkRieex8wD8vq5MTahYK2UeRXAODvLE6NsZ/89ckvXlkMgAgRqlP/se9OTW0er/VNUu0M/eCAGXWgs0ji6it3/MAVY5tC4T5h/Inzn1NZQDa0XYMzCE2z5ionyuw/Vwk6rcqOsVqpxOYsLGMvcE++a64J8WJBoQHR9zSnStw++AarPQgqL0z9bQFbpopm2bw09FyY47tp4YvKl1kyYmPt0zbToit+D+r+WWQF0/1k1ikMaUt4SOhTRXHz/ZrzRW1jtnJOkYiCJ1qsxH8dH+2sS3dtrX1hzIZ+mKu4o+x1jsYClXc0P9p5x7bR1HGOH/cnsKLiBBly2QW";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private Trajectory goToRings, zero, one, four, getSecondWobbleZero, deliverSecondWobbleZero,
            getSecondWobbleOne, deliverSecondWobbleOne, getSecondWobbleFour, deliverSecondWobbleFour;

    public static int result = 0;

    public static double wobbleClawOpen = 1.0;
    public static double wobbleClawClose = 0.0;

    public static int wobbleArmDistance = 420;

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
        goToRings = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-23.0, -58.0, Math.toRadians(0.0))) // -23.0, -53.0
                .build();

        zero = drive.trajectoryBuilder(goToRings.end())
                .lineToConstantHeading(new Vector2d(9.0, -49.0))
                .build();

        one = drive.trajectoryBuilder(goToRings.end())
                .lineToConstantHeading(new Vector2d(41.0, -27.0))
                .build();

        four = drive.trajectoryBuilder(goToRings.end())
                .lineToConstantHeading(new Vector2d(12.0, -55.0))
                .splineToConstantHeading(new Vector2d(36.5, -55.0), Math.toRadians(0.0))
                .splineToConstantHeading(new Vector2d(63.0, -55.0), Math.toRadians(0.0))
                .build();

        getSecondWobbleZero = drive.trajectoryBuilder(new Pose2d(zero.end().getX(), zero.end().getY(), Math.toRadians(0.0)))
                .lineToConstantHeading(new Vector2d(-12.0, -50.0))
                .splineToConstantHeading(new Vector2d(-54.0, -20.0), Math.toRadians(180.0))
                //.lineToLinearHeading(new Pose2d(-54.0, -40.0, Math.toRadians(180.0)))
                .build();

        deliverSecondWobbleZero = drive.trajectoryBuilder(getSecondWobbleZero.end())
                .lineToConstantHeading(new Vector2d(12.0, -48.0))
                .build();

        getSecondWobbleOne = drive.trajectoryBuilder(one.end())
                .lineToConstantHeading(new Vector2d(-48.0, -12.0))
                .build();

        deliverSecondWobbleOne = drive.trajectoryBuilder(getSecondWobbleOne.end())
                .lineToConstantHeading(new Vector2d(35.0, -20.0))
                .build();

        getSecondWobbleFour = drive.trajectoryBuilder(four.end())
                .lineToConstantHeading(new Vector2d(-50.0, -40.0))
                .build();

        deliverSecondWobbleFour = drive.trajectoryBuilder(getSecondWobbleFour.end())
                .lineToConstantHeading(new Vector2d(12.0, -42.0))
                .splineToConstantHeading(new Vector2d(33.5, -42.0), Math.toRadians(0.0))
                .splineToConstantHeading(new Vector2d(60.5, -42.0), Math.toRadians(0.0))
                .build();

        telemetry.addData(">", "Press Play to begin autonomous");
        telemetry.update();

        waitForStart();

        if(running()) {

            drive.followTrajectory(goToRings);

            sleep(500);

            if(tfod != null) {
                result = robotControl.detectRings(tfod, (MultipleTelemetry) telemetry);
            }

            telemetry.addData("Scan complete", result);
            telemetry.update();

            switch(result) {
                case 0:
                    drive.followTrajectory(zero);
                    moveWobbleArm(drive, 0.4, wobbleArmDistance);
                    sleep(400);
                    drive.wobbleClaw.setPosition(wobbleClawOpen);
                    sleep(300);
                    moveWobbleArm(drive, 0.4, -wobbleArmDistance);
                    drive.turn(Math.toRadians(180.0));

                    drive.followTrajectory(getSecondWobbleZero);
                    moveWobbleArm(drive, 0.4, wobbleArmDistance);
                    sleep(400);
                    drive.wobbleClaw.setPosition(wobbleClawClose);
                    sleep(600);
                    moveWobbleArm(drive, 0.4, -wobbleArmDistance);

                    drive.followTrajectory(deliverSecondWobbleZero);
                    drive.turn(Math.toRadians(180.0));
                    moveWobbleArm(drive, 0.4, wobbleArmDistance);
                    drive.wobbleClaw.setPosition(wobbleClawOpen);
                    sleep(150);
                    moveWobbleArm(drive, 0.4, -wobbleArmDistance);
                    break;
                case 1:
                    drive.followTrajectory(one);
                    moveWobbleArm(drive, 0.4, wobbleArmDistance);
                    sleep(400);
                    drive.wobbleClaw.setPosition(wobbleClawOpen);
                    sleep(300);
                    moveWobbleArm(drive, 0.4, -wobbleArmDistance);

                    drive.followTrajectory(getSecondWobbleOne);
                    moveWobbleArm(drive, 0.4, wobbleArmDistance);
                    sleep(400);
                    drive.wobbleClaw.setPosition(wobbleClawClose);
                    sleep(600);
                    moveWobbleArm(drive, 0.4, -wobbleArmDistance);

                    drive.followTrajectory(deliverSecondWobbleOne);
                    moveWobbleArm(drive, 0.4, wobbleArmDistance);
                    drive.wobbleClaw.setPosition(wobbleClawOpen);
                    sleep(150);
                    moveWobbleArm(drive, 0.4, -wobbleArmDistance);

                    drive.followTrajectory(returnToBaseline(deliverSecondWobbleOne.end(), drive));
                    break;
                case 4:
                    drive.followTrajectory(four);
                    moveWobbleArm(drive, 0.4, 380);
                    drive.wobbleClaw.setPosition(wobbleClawOpen);
                    sleep(150);
                    moveWobbleArm(drive, 0.4, -380);

                    drive.followTrajectory(getSecondWobbleFour);
                    drive.turn(Math.toRadians(180.0));
                    moveWobbleArm(drive, 0.4, 380);
                    sleep(150);
                    drive.wobbleClaw.setPosition(wobbleClawClose);
                    sleep(100);
                    moveWobbleArm(drive, 0.4, -380);

                    drive.followTrajectory(deliverSecondWobbleFour);
                    drive.turn(Math.toRadians(180.0));
                    moveWobbleArm(drive, 0.4, 380);
                    drive.wobbleClaw.setPosition(wobbleClawOpen);
                    sleep(150);
                    moveWobbleArm(drive, 0.4, -380);

                    drive.followTrajectory(returnToBaseline(deliverSecondWobbleFour.end(), drive));
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
