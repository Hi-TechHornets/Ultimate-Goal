package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotControl;
import org.firstinspires.ftc.teamcode.util.ToggleBoolean;

@TeleOp
@Config
public class moveAndShoot extends LinearOpMode {
    private robotControl tbd;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public float x, y, z, w, pwr;
    public float a, b, c;
    public static double deadzone = 0.14;

    // Defined hardware values

    public static double flickOpen = 0.85;
    public static double flickClose = 0.62;
    public static double wobbleClawOpen = 1.0;
    public static double wobbleClawClose = 0.0;

    public static double shooterPower = -0.62;

    public ToggleBoolean flickState;
    public ToggleBoolean shootState;
    public ToggleBoolean clawState;

    public void runOpMode() {
        clawState = new ToggleBoolean();
        flickState = new ToggleBoolean();
        shootState = new ToggleBoolean();

        tbd = new robotControl();

        tbd.init(hardwareMap);

        if(dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }

        tbd.wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tbd.wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tbd.wobbleClaw.setPosition(wobbleClawClose);

        tbd.flicker.setPosition(flickClose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Status", "Running");

            getJoyVals();

            pwr = y;

            tbd.rightFront.setPower(Range.clip(pwr - x + z, -1, 1));
            tbd.leftRear.setPower(Range.clip(pwr - x - z, -1, 1));
            tbd.leftFront.setPower(Range.clip(pwr + x - z, -1, 1));
            tbd.rightRear.setPower(Range.clip(pwr + x + z, -1, 1));

            if(clawState.output()) {
                tbd.wobbleClaw.setPosition(wobbleClawOpen);
            }
            else {
                tbd.wobbleClaw.setPosition(wobbleClawClose);
            }

            if(flickState.output()) {
                tbd.flicker.setPosition(flickOpen);
            }
            else {
                tbd.flicker.setPosition(flickClose);
            }

            if(shootState.output()) {
                tbd.shooter.setPower(shooterPower);
            }
            else {
                tbd.shooter.setPower(0.0);
            }

            tbd.intake.setPower(a);

            tbd.wobbleArm.setPower(b * 0.4);

            telemetry.addData("Motor position", tbd.wobbleArm.getCurrentPosition());
            telemetry.update();
        }
    }

    private void getJoyVals() {
        y = gamepad1.left_stick_y;
        x = -gamepad1.left_stick_x;
        z = gamepad1.right_stick_x;
        w = gamepad1.right_stick_y;

        a = gamepad2.left_stick_y;
        b = gamepad2.right_stick_y;

        if(Math.abs(-x) < deadzone) x = 0;
        if(Math.abs(y) < deadzone) y = 0;
        if(Math.abs(z) < deadzone) z = 0;
        if(Math.abs(w) < 0.9) w = 0;

        if(Math.abs(a) < deadzone) a = 0;
        if(Math.abs(b) < deadzone) b = 0;

        clawState.input(gamepad2.x);

        flickState.input(gamepad2.a);
        shootState.input(gamepad2.b);
    }
}