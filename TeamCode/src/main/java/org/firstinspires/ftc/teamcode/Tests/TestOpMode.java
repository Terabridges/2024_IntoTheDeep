package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ServoHubConfiguration;

import org.firstinspires.ftc.teamcode.Utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.Utility.EdgeDetector;

@TeleOp(name="TestOpMode", group="Test")
@Config
public class TestOpMode extends LinearOpMode {

    public CRServo intakeLeftSlide;
    public CRServo intakeRightSlide;
    public CRServo intakeRightSwivel;
    public CRServo intakeLeftSwivel;
    public DcMotor intakeSpin;
    public AnalogInput intakeRightSwivelAnalog;
    public AnalogInput intakeRightSlidesAnalog;
    public AbsoluteAnalogEncoder intakeRightSwivelEnc;
    public AbsoluteAnalogEncoder intakeRightSlidesEnc;
    public DcMotor outtakeTopVertical;
    public DcMotor outtakeBottomVertical;
    public CRServo outtakeLeftSwivel;
    public CRServo outtakeRightSwivel;
    public Servo outtakeWrist;
    public Servo outtakeClaw;
    public AnalogInput outtakeRightSwivelAnalog;
    public AbsoluteAnalogEncoder outtakeRightSwivelEnc;
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;
    public Servo intakeSweeper;

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();

    public static double CLAW_OPEN = 0.65;
    public static double CLAW_CLOSE = 0.52;
    public static double WRIST_UP = 0.59;
    public static double WRIST_DOWN = 0.25;
    public static double SWEEPER_OUT = 0.6;
    public static double SWEEPER_IN = 0.2;

    public boolean clawOpen = false;
    public boolean wristUp = false;
    public boolean sweeperOut = false;

    public enum Mode {
        INTAKE,
        OUTTAKE,
        SPIN,
        DRIVE
    }

    @Override
    public void runOpMode(){

        intakeLeftSlide = hardwareMap.get(CRServo.class, "intake_left_slide");
        intakeLeftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRightSlide = hardwareMap.get(CRServo.class, "intake_right_slide");
        intakeLeftSwivel = hardwareMap.get(CRServo.class, "intake_left_swivel");
        intakeLeftSwivel.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRightSwivel = hardwareMap.get(CRServo.class, "intake_right_swivel");
        intakeSpin = hardwareMap.get(DcMotor.class, "intake_spin");
        intakeRightSwivelAnalog = hardwareMap.get(AnalogInput.class, "intake_right_swivel_analog");
        intakeRightSwivelEnc = new AbsoluteAnalogEncoder(intakeRightSwivelAnalog);
        intakeRightSlidesAnalog = hardwareMap.get(AnalogInput.class, "intake_right_slide_analog");
        intakeRightSlidesEnc = new AbsoluteAnalogEncoder(intakeRightSlidesAnalog);
        outtakeTopVertical = hardwareMap.get(DcMotor.class, "outtake_bottom_vertical");
        outtakeBottomVertical = hardwareMap.get(DcMotor.class, "outtake_top_vertical");
        outtakeBottomVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeLeftSwivel = hardwareMap.get(CRServo.class, "outtake_left_swivel");
        outtakeLeftSwivel.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeRightSwivel = hardwareMap.get(CRServo.class, "outtake_right_swivel");
        outtakeWrist = hardwareMap.get(Servo.class, "outtake_wrist");
        outtakeClaw = hardwareMap.get(Servo.class, "outtake_claw");
        outtakeRightSwivelAnalog = hardwareMap.get(AnalogInput.class, "outtake_right_swivel_analog");
        outtakeRightSwivelEnc = new AbsoluteAnalogEncoder(outtakeRightSwivelAnalog);
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        intakeSweeper = hardwareMap.get(Servo.class, "intake_sweeper");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Mode mode = Mode.INTAKE;

        waitForStart();

        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            switch (mode){
                case INTAKE:
                    intakeSetSlides(gamepad1.left_stick_y);
                    intakeSetSwivel(gamepad1.right_stick_y);
                    break;
                case OUTTAKE:
                    outtakeSetSlides(gamepad1.left_stick_y);
                    outtakeSetSwivel(gamepad1.right_stick_y);
                    break;
                case SPIN:
                    intakeSetSpin(gamepad1.left_stick_y);
                    break;
                case DRIVE:
                    double max;
                    // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                    double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                    double lateral = gamepad1.left_stick_x;
                    double yaw = gamepad1.right_stick_x;
                    // Combine the joystick requests for each axis-motion to determine each wheel's power.
                    // Set up a variable for each drive wheel to save the power level for telemetry.
                    double leftFrontPower = axial + lateral + yaw;
                    double rightFrontPower = axial - lateral - yaw;
                    double leftBackPower = axial - lateral + yaw;
                    double rightBackPower = axial + lateral - yaw;
                    // Normalize the values so no wheel power exceeds 100%
                    // This ensures that the robot maintains the desired motion.
                    max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                    max = Math.max(max, Math.abs(leftBackPower));
                    max = Math.max(max, Math.abs(rightBackPower));
                    if (max > 1.0) {
                        leftFrontPower /= max;
                        rightFrontPower /= max;
                        leftBackPower /= max;
                        rightBackPower /= max;
                    }
                    leftFront.setPower(leftFrontPower);
                    rightFront.setPower(rightFrontPower);
                    leftBack.setPower(leftBackPower);
                    rightBack.setPower(rightBackPower);
                    break;
            }

            if (currentGamepad1.a && !previousGamepad1.a){
                switch (mode){
                    case INTAKE:
                        mode = Mode.OUTTAKE;
                        break;
                    case OUTTAKE:
                        mode = Mode.SPIN;
                        break;
                    case SPIN:
                        mode = Mode.DRIVE;
                        break;
                    case DRIVE:
                        mode = Mode.INTAKE;
                        break;
                }
            }

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                outtakeClaw.setPosition((clawOpen ? CLAW_CLOSE : CLAW_OPEN));
                clawOpen = !clawOpen;
            }

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                outtakeWrist.setPosition((wristUp ? WRIST_DOWN : WRIST_UP));
                wristUp = !wristUp;
            }

            if (currentGamepad1.left_stick_button && !previousGamepad1.left_stick_button){
                intakeSweeper.setPosition((sweeperOut ? SWEEPER_IN : SWEEPER_OUT));
                sweeperOut = !sweeperOut;
            }

            telemetry.addData("Current Mode: ", mode);
            telemetry.addData("Outtake swivel abs pos", outtakeRightSwivelEnc.getCurrentPosition());
            telemetry.addData("Intake swivel abs pos", intakeRightSwivelEnc.getCurrentPosition());
            telemetry.addData("Intake slides abs pos", intakeRightSlidesEnc.getCurrentPosition());
            telemetry.addData("Outtake swivel pos", outtakeRightSwivelAnalog.getVoltage());
            telemetry.addData("Intake swivel pos", intakeRightSwivelAnalog.getVoltage());
            telemetry.addData("Intake slides pos", intakeRightSlidesAnalog.getVoltage());
            telemetry.addData("Is Right Bumper", gamepad1.right_bumper);
            telemetry.update();
        }


    }

    public void intakeSetSlides(double pow) {
        intakeLeftSlide.setPower(pow);
        intakeRightSlide.setPower(pow);
    }

    public void intakeSetSwivel(double pow) {
        intakeLeftSwivel.setPower(pow);
        intakeRightSwivel.setPower(pow);
    }

    public void intakeSetSpin(double pow) {
        intakeSpin.setPower(pow);
    }

    public void outtakeSetSlides(double pow) {
        outtakeBottomVertical.setPower(pow);
        outtakeTopVertical.setPower(pow);
    }

    public void outtakeSetSwivel(double pow) {
        outtakeLeftSwivel.setPower(pow);
        outtakeRightSwivel.setPower(pow);
    }
}
