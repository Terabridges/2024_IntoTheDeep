package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
@TeleOp(name = "MotorTest", group = "Test")
@Config
public class MotorTest extends LinearOpMode {

        private DcMotor outtakeBottomVertical;
        private DcMotor outtakeMiddleVertical;
        private DcMotor outtakeTopVertical;

        private int outtakeMode = 0;

        public Gamepad currentGamepad1;
        public Gamepad previousGamepad1;

        @Override
        public void runOpMode() {

            outtakeTopVertical = hardwareMap.get(DcMotor.class, "outtake_top_vertical");
            outtakeBottomVertical = hardwareMap.get(DcMotor.class, "outtake_bottom_vertical");
            outtakeMiddleVertical = hardwareMap.get(DcMotor.class, "outtake_middle_vertical");
            outtakeBottomVertical.setDirection(DcMotor.Direction.REVERSE);
            outtakeTopVertical.setDirection(DcMotor.Direction.REVERSE);

            currentGamepad1 = new Gamepad();
            previousGamepad1 = new Gamepad();

            waitForStart();

            while (opModeIsActive()){
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                switch (outtakeMode){
                    case 0:
                        telemetry.addData("Mode", "Bottom");
                        outtakeBottomVertical.setPower(gamepad1.left_stick_y);
                        break;
                    case 1:
                        telemetry.addData("Mode", "Middle");
                        outtakeMiddleVertical.setPower(gamepad1.left_stick_y);
                        break;
                    case 2:
                        telemetry.addData("Mode", "Top");
                        outtakeTopVertical.setPower(gamepad1.left_stick_y);
                        break;
                    case 3:
                        telemetry.addData("Mode", "All");
                        outtakeBottomVertical.setPower(gamepad1.left_stick_y);
                        outtakeMiddleVertical.setPower(gamepad1.left_stick_y);
                        outtakeTopVertical.setPower(gamepad1.left_stick_y);
                        break;
                }

                if (currentGamepad1.a && !previousGamepad1.a){
                    switch (outtakeMode){
                        case 0:
                            outtakeMode = 1;
                            break;
                        case 1:
                            outtakeMode = 2;
                            break;
                        case 2:
                            outtakeMode = 3;
                            break;
                        case 3:
                            outtakeMode = 0;
                            break;
                    }
                }
                telemetry.update();
            }
        }
}
