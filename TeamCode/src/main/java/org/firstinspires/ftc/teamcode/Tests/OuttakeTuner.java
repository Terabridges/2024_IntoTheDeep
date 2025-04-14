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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Utility.AbsoluteAnalogEncoder;

@Disabled
@TeleOp(name="OuttakeTuner", group="Test")
@Config
public class OuttakeTuner extends LinearOpMode {

    public DcMotor outtakeTopVertical;
    public DcMotor outtakeBottomVertical;
    public DcMotor outtakeMiddleVertical;
    public CRServo outtakeLeftSwivel;
    public CRServo outtakeRightSwivel;
    public Servo outtakeWrist;
    public Servo outtakeClaw;
    public AnalogInput outtakeRightSwivelAnalog;
    public AbsoluteAnalogEncoder outtakeRightSwivelEnc;
    public TouchSensor limit;

    private double ticks_in_degree = 144.0 / 180.0;

    public Gamepad currentGamepad1;
    public Gamepad previousGamepad1;

    //Third PID for outtake slides
    private PIDController outtakeSlidesController;
    public static double p3 = 0.0021, i3 = 0.1, d3 = 0.0001;
    public static double f3 = -0.06;
    public static int outtakeSlidesTarget;
    double outtakeSlidesPos;
    double pid3, targetOuttakeSlidesAngle, ff3, currentOuttakeSlidesAngle, outtakeSlidesPower;

    //Fourth PID for outtake swivel
    private PIDController outtakeSwivelController;
    public static double p4 = 0.003, i4 = 0.001, d4 = 0.00005;
    public static double f4 = -0.02;
    public static int outtakeSwivelTarget;
    double outtakeSwivelPos;
    double pid4, targetOuttakeSwivelAngle, ff4, currentOuttakeSwivelAngle, outtakeSwivelPower;

    public boolean runSlides = false;
    public static double outtakeSwivelOffset = -5;
    public double outtakeSwivelGearRatio = 40.0 / 30.0;

    public int highLimit = -1600;

    @Override
    public void runOpMode() {

        outtakeTopVertical = hardwareMap.get(DcMotor.class, "outtake_top_vertical");
        outtakeBottomVertical = hardwareMap.get(DcMotor.class, "outtake_bottom_vertical");
        outtakeMiddleVertical = hardwareMap.get(DcMotor.class, "outtake_middle_vertical");
        outtakeTopVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeBottomVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeLeftSwivel = hardwareMap.get(CRServo.class, "outtake_left_swivel");
        outtakeLeftSwivel.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeRightSwivel = hardwareMap.get(CRServo.class, "outtake_right_swivel");
        outtakeWrist = hardwareMap.get(Servo.class, "outtake_wrist");
        outtakeClaw = hardwareMap.get(Servo.class, "outtake_claw");
        outtakeRightSwivelAnalog = hardwareMap.get(AnalogInput.class, "outtake_right_swivel_analog");
        outtakeRightSwivelEnc = new AbsoluteAnalogEncoder(outtakeRightSwivelAnalog, 3.3, outtakeSwivelOffset, outtakeSwivelGearRatio);
        limit = hardwareMap.get(TouchSensor.class, "limit");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outtakeSlidesController = new PIDController(p3, i3, d3);
        outtakeSwivelController = new PIDController(p4, i4, d4);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        waitForStart();

        while (opModeIsActive()){

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

//            if (limit.isPressed()){
//                outtakeMiddleVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }

            if (currentGamepad1.a && !previousGamepad1.a){
                runSlides = !runSlides;
            }

            if (runSlides) {
                outtakeSlidesSetPower(setOuttakeSlidesPIDF(outtakeSlidesTarget));
            } else {
                outtakeSwivelSetPower(setOuttakeSwivelPIDF(outtakeSwivelTarget));
            }

            telemetry.addData("Running Slides: ", runSlides);

            if (runSlides) {
                telemetry.addData("Linear Slides Target", outtakeSlidesTarget);
                telemetry.addData("Linear Slides Pos", outtakeMiddleVertical.getCurrentPosition());
            } else {
                telemetry.addData("Swivel Target", outtakeSwivelTarget);
                telemetry.addData("Swivel Pos", outtakeRightSwivelEnc.getCurrentPosition());
            }

            //telemetry.addData("LIMITING?", limit.isPressed() ? "YES" : "NO");

            telemetry.update();

        }
    }

    public double setOuttakeSlidesPIDF(int target) {
        outtakeSlidesController.setPID(p3, i3, d3);
        outtakeSlidesPos = outtakeMiddleVertical.getCurrentPosition();
        pid3 = outtakeSlidesController.calculate(outtakeSlidesPos, target);
        targetOuttakeSlidesAngle = target;
        ff3 = f3;
        currentOuttakeSlidesAngle = Math.toRadians((outtakeSlidesPos) / ticks_in_degree);

        outtakeSlidesPower = pid3 + ff3;

        return outtakeSlidesPower;
    }

    public double setOuttakeSwivelPIDF(int target) {
        outtakeSwivelController.setPID(p4, i4, d4);
        outtakeSwivelPos = outtakeRightSwivelEnc.getCurrentPosition();
        pid4 = outtakeSwivelController.calculate(outtakeSwivelPos, target);
        targetOuttakeSwivelAngle = target;
        ff4 = (Math.sin(Math.toRadians(targetOuttakeSwivelAngle))) * f4;
        currentOuttakeSwivelAngle = Math.toRadians((outtakeSwivelPos) / ticks_in_degree);

        outtakeSwivelPower = pid4 + ff4;

        return outtakeSwivelPower;
    }

    public void outtakeSlidesSetPower(double pow){
        telemetry.addData("Power", pow);
        if (!(outtakeMiddleVertical.getCurrentPosition() < highLimit+20 && pow < 0)){
            outtakeTopVertical.setPower(pow);
            outtakeBottomVertical.setPower(pow);
            outtakeMiddleVertical.setPower(pow);
        } else {
            outtakeSlidesTarget = highLimit;
        }
        telemetry.update();
    }
    public void outtakeSwivelSetPower(double pow){
        outtakeLeftSwivel.setPower(pow);
        outtakeRightSwivel.setPower(pow);
    }
}
