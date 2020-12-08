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
public class teleOp extends LinearOpMode {
    private robotControl tbd;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public float x, y, z, w, pwr;
    public float a, b, c;
    public static double deadzone = 0.14;

    // Defined hardware values

    public static double wobbleClawOpen = 1.0;
    public static double wobbleClawClose = 0.0;

    public ToggleBoolean clawState;

    public void runOpMode() {
        clawState = new ToggleBoolean();

        tbd = new robotControl();

        tbd.init(hardwareMap);

        if(dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }

        tbd.wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tbd.wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tbd.wobbleClaw.setPosition(wobbleClawClose);

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

            tbd.wobbleArm.setPower(a * 0.4);

//            tbd.rightWheel.setPower(b);
//            tbd.leftWheel.setPower(c);

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
//        b = gamepad2.right_stick_y;
//        c = gamepad2.left_stick_y;

        if(Math.abs(-x) < deadzone) x = 0;
        if(Math.abs(y) < deadzone) y = 0;
        if(Math.abs(z) < deadzone) z = 0;
        if(Math.abs(w) < 0.9) w = 0;

        if(Math.abs(a) < deadzone) a = 0;
//        if(Math.abs(b) < deadzone) b = 0;
//        if(Math.abs(c) < deadzone) c = 0;

        clawState.input(gamepad2.a);

        telemetry.addData("ly", y);
        telemetry.addData("lx", x);
        telemetry.addData("lx_raw", gamepad1.left_stick_x);
        telemetry.addData("rx", z);

//        out.input(gamepad2.b);
//        in.input(gamepad2.a);
//
//        mode.input(gamepad1.x);
//        speedMode.input(gamepad1.y);
//
//        rotateState.input(gamepad2.y);
//        gripperState.input(gamepad2.x);
    }
}
