package org.firstinspires.ftc.teamcode.Tests;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.ServoHubConfiguration;

@TeleOp(name="NetZoneAlignTest", group="Test")
@Config

public class NetZoneAlignTest extends LinearOpMode {

    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;
    public AnalogInput leftBackDistance;
    public AnalogInput rightBackDistance;
    private Follower follower;



    @Override
    public void runOpMode() {
        leftBack = hardwareMap.get(DcMotor .class, "left_back");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDistance = hardwareMap.get(AnalogInput.class, "left_back_distance");
        rightBackDistance = hardwareMap.get(AnalogInput.class, "right_back_distance");
        follower = new Follower(hardwareMap);

        waitForStart();
        
        while (opModeIsActive()) {
        }
    }
}
