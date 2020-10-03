package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotControl;
import org.firstinspires.ftc.teamcode.util.ToggleBoolean;

@TeleOp
public class wobbleTest extends LinearOpMode {
    private robotControl tbd;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public float x, y, z, w, pwr;
    public float a;
    public static double deadzone = 0.2;

    // Defined hardware values

    public static double wobbleClawOpen = 1.0;
    public static double wobbleClawClose = 0.1;

    public ToggleBoolean clawState;

    public void runOpMode() {
        clawState = new ToggleBoolean();

        tbd = new robotControl();

        tbd.init(hardwareMap);

        if(dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Status", "Running");

            getJoyVals();

            pwr = y;

            tbd.moveDriveMotors(Range.clip(pwr - x + z, -1, 1),
                                Range.clip(pwr - x - z, -1, 1),
                                Range.clip(pwr - x + z, -1, 1),
                                Range.clip(pwr + x + z, -1, 1));

            if(clawState.output()) {
                tbd.wobbleClaw.setPosition(wobbleClawClose);
            }
            else {
                tbd.wobbleClaw.setPosition(wobbleClawOpen);
            }

            tbd.wobbleArm.setPower(a * 0.4);

            telemetry.update();
        }
    }

    private void getJoyVals() {
        y = -gamepad1.left_stick_y;
        x = -gamepad1.left_stick_x;
        z = gamepad1.right_stick_x;
        w = gamepad1.right_stick_y;

        a = gamepad2.left_stick_y;

        if(Math.abs(-x) < deadzone) x = 0;
        if(Math.abs(y) < deadzone) y = 0;
        if(Math.abs(z) < deadzone) z = 0;
        if(Math.abs(w) < 0.9) w = 0;

        if(Math.abs(a) < deadzone) a = 0;

        clawState.input(gamepad2.a);

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
