package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utility.AbsoluteAnalogEncoder;

@Config
public class OuttakeTuner extends LinearOpMode {

    public DcMotor outtakeTopVertical;
    public DcMotor outtakeBottomVertical;
    public CRServo outtakeLeftSwivel;
    public CRServo outtakeRightSwivel;
    public Servo outtakeWrist;
    public Servo outtakeClaw;
    public AnalogInput outtakeRightSwivelAnalog;
    public AbsoluteAnalogEncoder outtakeRightSwivelEnc;

    private double ticks_in_degree = 144.0 / 180.0;

    //Third PID for outtake slides
    private PIDController outtakeSlidesController;
    public double p3 = 0.005, i3 = 0.02, d3 = 0.00004;
    public double f3 = 0.06;
    public int outtakeSlidesTarget;
    double outtakeSlidesPos;
    double pid3, targetOuttakeSlidesAngle, ff3, currentOuttakeSlidesAngle, outtakeSlidesPower;

    //Fourth PID for outtake swivel
    private PIDController outtakeSwivelController;
    public double p4 = 0.005, i4 = 0.02, d4 = 0.00004;
    public double f4 = 0.06;
    public int outtakeSwivelTarget;
    double outtakeSwivelPos;
    double pid4, targetOuttakeSwivelAngle, ff4, currentOuttakeSwivelAngle, outtakeSwivelPower;

    @Override
    public void runOpMode() {

        outtakeTopVertical = hardwareMap.get(DcMotor.class, "outtake_top_vertical");
        outtakeBottomVertical = hardwareMap.get(DcMotor.class, "outtake_bottom_vertical");
        outtakeLeftSwivel = hardwareMap.get(CRServo.class, "outtake_left_swivel");
        outtakeRightSwivel = hardwareMap.get(CRServo.class, "outtake_right_swivel");
        outtakeWrist = hardwareMap.get(Servo.class, "outtake_wrist");
        outtakeClaw = hardwareMap.get(Servo.class, "outtake_claw");
        outtakeRightSwivelAnalog = hardwareMap.get(AnalogInput.class, "outtake_right_swivel_analog");
        outtakeRightSwivelEnc = new AbsoluteAnalogEncoder(outtakeRightSwivelAnalog, 3.3, 0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outtakeSlidesController = new PIDController(p3, i3, d3);
        outtakeSwivelController = new PIDController(p4, i4, d4);

        waitForStart();
        outtakeTopVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive()){

        }
    }
}
