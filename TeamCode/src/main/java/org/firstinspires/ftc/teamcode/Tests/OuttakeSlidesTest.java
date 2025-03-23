package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@Disabled
@TeleOp(name="SlidesTest", group="Test")
@Config
public class OuttakeSlidesTest extends LinearOpMode {
    public DcMotor outtakeTopVertical;
    public DcMotor outtakeBottomVertical;

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();

    public enum Mode {
        BOTH,
        TOP,
        BOTTOM
    }

    @Override
    public void runOpMode() {
        outtakeTopVertical = hardwareMap.get(DcMotor.class, "outtake_top_vertical");
        outtakeBottomVertical = hardwareMap.get(DcMotor.class, "outtake_bottom_vertical");
        outtakeTopVertical.setDirection(DcMotorSimple.Direction.REVERSE);

        Mode mode = Mode.BOTH;

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            switch (mode) {
                case BOTH:
                    outtakeBottomVertical.setPower(gamepad1.left_stick_y);
                    outtakeTopVertical.setPower(gamepad1.left_stick_y);
                    break;
                case TOP:
                    outtakeTopVertical.setPower(gamepad1.left_stick_y);
                    break;
                case BOTTOM:
                    outtakeBottomVertical.setPower(gamepad1.left_stick_y);
                    break;
            }

            if (currentGamepad1.a && !previousGamepad1.a) {
                switch (mode) {
                    case BOTH:
                        mode = Mode.TOP;
                        break;
                    case TOP:
                        mode = Mode.BOTTOM;
                        break;
                    case BOTTOM:
                        mode = Mode.BOTH;
                        break;
                }
            }

            telemetry.addData("Currently", mode);
            telemetry.update();
        }
    }
}
