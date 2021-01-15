package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotControl;
import org.firstinspires.ftc.teamcode.util.ToggleBoolean;

@TeleOp
public class shootingSystemTest extends LinearOpMode {
    private robotControl tbd;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public float x, y, z, w, pwr;
    public float a, b, c;
    public static double deadzone = 0.14;

    // Defined hardware values

    public static double flickOpen = 0.85;
    public static double flickClose = 0.62;

    public ToggleBoolean flickState;
    public ToggleBoolean shootState;

    public void runOpMode() {

        flickState = new ToggleBoolean();
        shootState = new ToggleBoolean();

        tbd = new robotControl();

        tbd.init(hardwareMap);

        if(dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }

        tbd.flicker.setPosition(flickClose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Status", "Running");

            getJoyVals();

            if(flickState.output()) {
                tbd.flicker.setPosition(flickOpen);
            }
            else {
                tbd.flicker.setPosition(flickClose);
            }

            if(shootState.output()) {
                tbd.shooter.setPower(1.0);
            }
            else {
                tbd.shooter.setPower(0.0);
            }

            tbd.intake.setPower(a);

            telemetry.update();
        }
    }

    private void getJoyVals() {
        a = gamepad1.left_stick_y;


        flickState.input(gamepad1.a);
        shootState.input(gamepad1.b);

        if(Math.abs(a) < deadzone) a = 0;
    }
}
