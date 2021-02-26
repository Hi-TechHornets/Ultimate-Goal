package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;

@Autonomous
@Config
public class autoShoot extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double wobbleClawOpen = 1.0;
    public static double wobbleClawClose = 0.0;

    public static int wobbleArmDistance = 420;

    public static double shooterPower = -0.67;
    public static double flickOpen = 0.85;
    public static double flickClose = 0.62;
    public static int delay = 250;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }
        else {
            telemetry = new MultipleTelemetry(telemetry);
        }

        telemetry.addData(">", "Press Play to begin autonomous");
        telemetry.update();

        waitForStart();

        if(running()) {
            drive.shooter.setPower(shooterPower);
            sleep(1200);
            // move servo back and forth fast
            for(int i = 0; i < 3; i++) {
                drive.flicker.setPosition(flickOpen);
                sleep(delay);
                drive.flicker.setPosition(flickClose);
                sleep(delay);
            }
            drive.shooter.setPower(0.0);
        }
    }

    public boolean running() {
        return opModeIsActive() && !isStopRequested();
    }
}

