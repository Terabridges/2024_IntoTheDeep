package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Utility.AbsoluteAnalogEncoder;

//@Disabled
@TeleOp(name="IntakeTuner", group="Test")
@Config
public class IntakeTuner extends LinearOpMode {

    public CRServo intakeLeftSlide;
    public CRServo intakeRightSlide;
    public CRServo intakeRightSwivel;
    public CRServo intakeLeftSwivel;
    public DcMotor intakeSpin;
    public AnalogInput intakeRightSwivelAnalog;
    public AnalogInput intakeRightSlidesAnalog;
    public AbsoluteAnalogEncoder intakeRightSwivelEnc;
    public AbsoluteAnalogEncoder intakeRightSlidesEnc;
//    public AnalogInput intakeLeftSwivelAnalog;
//    public AbsoluteAnalogEncoder intakeLeftSwivelEnc;

    private double ticks_in_degree = 144.0 / 180.0;

    //First PID for intake slides
    private PIDController intakeSlidesController;
    public static double p = 0.0075, i = 0.04, d = 0.0003;
    public static double f = 0.0;
    public static int intakeSlidesTarget;
    double intakeSlidesPos;
    double pid, targetIntakeSlidesAngle, ff, currentIntakeSlidesAngle, intakeSlidesPower;

    //Second PID for intake swivel
    private PIDController intakeSwivelController;
    public static double p2 = 0.005, i2 = 0.03, d2 = 0.00005;
    public static double f2 = 0.05;
    public static int intakeSwivelTarget;
    double intakeSwivelPos;
    double pid2, targetIntakeSwivelAngle, ff2, currentIntakeSwivelAngle, intakeSwivelPower;

    boolean runSlides = false;
    public static double intakeSwivelOffset = 190;
    public static double intakeSlidesOffset = 100;
    public double intakeSwivelGearRatio = 40.0/48.0;
//    public static double otherIntakeSwivelOffset = 80;

    public Gamepad currentGamepad1;
    public Gamepad previousGamepad1;

    @Override
    public void runOpMode() {
        intakeLeftSlide = hardwareMap.get(CRServo.class, "intake_left_slide");
        intakeLeftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRightSlide = hardwareMap.get(CRServo.class, "intake_right_slide");
        intakeLeftSwivel = hardwareMap.get(CRServo.class, "intake_left_swivel");
        intakeRightSwivel = hardwareMap.get(CRServo.class, "intake_right_swivel");
        intakeRightSwivel.setDirection(DcMotorSimple.Direction.REVERSE);
        //intakeLeftSwivel.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSpin = hardwareMap.get(DcMotor.class, "intake_spin");
        intakeRightSwivelAnalog = hardwareMap.get(AnalogInput.class, "intake_left_swivel_analog");
        intakeRightSlidesAnalog = hardwareMap.get(AnalogInput.class, "intake_right_slide_analog");
        intakeRightSlidesEnc = new AbsoluteAnalogEncoder(intakeRightSlidesAnalog, 3.3, intakeSlidesOffset, 1);
        intakeRightSwivelEnc = new AbsoluteAnalogEncoder(intakeRightSwivelAnalog, 3.3, intakeSwivelOffset, intakeSwivelGearRatio);
//        intakeRightSwivelEnc.setInverted(true);
//        intakeLeftSwivelAnalog = hardwareMap.get(AnalogInput.class, "intake_left_swivel_analog");
//        intakeLeftSwivelEnc = new AbsoluteAnalogEncoder(intakeLeftSwivelAnalog, 3.3, otherIntakeSwivelOffset, intakeSwivelGearRatio);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeSlidesController = new PIDController(p, i, d);
        intakeSwivelController = new PIDController(p2, i2, d2);

        waitForStart();

        while (opModeIsActive()){

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.a && !previousGamepad1.a){
                runSlides = !runSlides;
            }

            if (runSlides) {
                intakeSlidesSetPower(setIntakeSlidesPIDF(intakeSlidesTarget));
            } else {
                intakeSwivelSetPower(setIntakeSwivelPIDF(intakeSwivelTarget));
            }

            telemetry.addData("Running Slides: ", runSlides);

            if (runSlides) {
                telemetry.addData("Linear Slides Target", intakeSlidesTarget);
                telemetry.addData("Linear Slides Pos", intakeRightSlidesEnc.getCurrentPosition());
            } else {
                telemetry.addData("Swivel Target", intakeSwivelTarget);
                telemetry.addData("Swivel Pos", intakeRightSwivelEnc.getCurrentPosition());
//                telemetry.addData("Other Swivel Pos", intakeLeftSwivelEnc.getCurrentPosition());

            }

            telemetry.addData("swivel ff", ff2);

            telemetry.update();
        }
    }

    public double setIntakeSlidesPIDF(int target) {
        intakeSlidesController.setPID(p, i, d);
        intakeSlidesPos = intakeRightSlidesEnc.getCurrentPosition();
        pid = intakeSlidesController.calculate(intakeSlidesPos, target);
        targetIntakeSlidesAngle = target;
        ff = (Math.sin(Math.toRadians(targetIntakeSlidesAngle))) * f;
        currentIntakeSlidesAngle = Math.toRadians((intakeSlidesPos) / ticks_in_degree);

        intakeSlidesPower = pid + ff;

        return intakeSlidesPower;
    }

    public double setIntakeSwivelPIDF(int target) {
        intakeSwivelController.setPID(p2, i2, d2);
        intakeSwivelPos = intakeRightSwivelEnc.getCurrentPosition();
        pid2 = intakeSwivelController.calculate(intakeSwivelPos, target);
        targetIntakeSwivelAngle = target;
        ff2 = (Math.sin(Math.toRadians(targetIntakeSwivelAngle))) * f2;
        currentIntakeSwivelAngle = Math.toRadians((intakeSwivelPos) / ticks_in_degree);

        intakeSwivelPower = pid2 + ff2;

        return intakeSwivelPower;
    }

    public void intakeSlidesSetPower(double pow){
        intakeLeftSlide.setPower(pow);
        intakeRightSlide.setPower(pow);
    }
    public void intakeSwivelSetPower(double pow){
        intakeLeftSwivel.setPower(pow);
        intakeRightSwivel.setPower(pow);
    }

}
