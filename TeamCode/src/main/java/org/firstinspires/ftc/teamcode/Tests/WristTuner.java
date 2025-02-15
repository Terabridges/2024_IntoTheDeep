package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utility.AbsoluteAnalogEncoder;

@TeleOp(name="WristTuner", group="Test")
@Config
public class WristTuner extends LinearOpMode {

    public Servo outtakeWrist;
    public CRServo intakeRightSwivel;
    public CRServo intakeLeftSwivel;
    public AnalogInput intakeRightSwivelAnalog;
    public AbsoluteAnalogEncoder intakeRightSwivelEnc;

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();

    public static double WRIST_TARGET = 0;
    public int INTAKE_SWIVEL_TRANSFER = 121;
    private double ticks_in_degree = 144.0 / 180.0;
    public double intakeSwivelGearRatio = 40.0/48.0;

    //Second PID for intake swivel
    public PIDController intakeSwivelController;
    public static double p2 = 0.0035, i2 = 0.02, d2 = 0.00009;
    public static double f2 = 0.04;
    public static int intakeSwivelTarget = 121;
    double intakeSwivelPos;
    double pid2, targetIntakeSwivelAngle, ff2, currentIntakeSwivelAngle, intakeSwivelPower;

    @Override
    public void runOpMode(){

        outtakeWrist = hardwareMap.get(Servo.class, "outtake_wrist");
        intakeLeftSwivel = hardwareMap.get(CRServo.class, "intake_left_swivel");
        intakeRightSwivel = hardwareMap.get(CRServo.class, "intake_right_swivel");
        intakeLeftSwivel.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRightSwivelAnalog = hardwareMap.get(AnalogInput.class, "intake_right_swivel_analog");
        intakeRightSwivelEnc = new AbsoluteAnalogEncoder(intakeRightSwivelAnalog, 3.3, 87, intakeSwivelGearRatio);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intakeSwivelController = new PIDController(p2, i2, d2);

        waitForStart();

        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.a && !previousGamepad1.a){
                outtakeWrist.setPosition(WRIST_TARGET);
            }

            //intakeSetSwivel(setIntakeSwivelPIDF(intakeSwivelTarget));
        }

    }

    public void intakeSetSwivel(double pow) {
        intakeLeftSwivel.setPower(pow);
        intakeRightSwivel.setPower(pow);
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
}
