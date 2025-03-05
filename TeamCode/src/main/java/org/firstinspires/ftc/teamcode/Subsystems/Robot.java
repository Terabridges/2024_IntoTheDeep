package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.DriveMediator;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class Robot {

    //Objects
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Gamepad gp1;
    public Gamepad gp2;
    public static VoltageSensor voltageSensor;

    //Subsystems
    public IntakeSystem intakeSystem;
    public OuttakeSystem outtakeSystem;
    public VisionSystem visionSystem;
    public DriveSystem driveSystem;

    public DriveMediator dM;

    //Other
    public String currentState = "none";

    //Subsystem List
    public List<Subsystem> subsystems;

    //Constructors
    public Robot(HardwareMap map, Telemetry t, Gamepad gp1, Gamepad gp2){
        hardwareMap = map;
        telemetry = t;

        outtakeSystem = new OuttakeSystem(hardwareMap);
        intakeSystem = new IntakeSystem(hardwareMap);
        driveSystem = new DriveSystem(hardwareMap);
        visionSystem = new VisionSystem(hardwareMap);

        subsystems = new ArrayList<>(Arrays.asList(outtakeSystem, intakeSystem, visionSystem, driveSystem));

        this.gp1 = gp1;
        this.gp2 = gp2;

        this.dM = new DriveMediator(this);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public Robot(HardwareMap map, Telemetry t){this(map, t, null, null);}

    //Methods
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    public void setManualSlidesTrue(){
        if (!outtakeSystem.manualOuttake) {
            outtakeSystem.manualOuttake = true;
            intakeSystem.manualIntake = false;
        } else {
            outtakeSystem.manualOuttake = false;
            intakeSystem.manualIntake = true;
            outtakeSystem.outtakeSlidesTarget = outtakeSystem.outtakeBottomVertical.getCurrentPosition();
        }
    }

    public void slowFall(){
        outtakeSystem.outtakeTopVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeSystem.outtakeBottomVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void regularFall(){
        outtakeSystem.outtakeTopVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeSystem.outtakeBottomVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void toggleLimitSwitch(){
        outtakeSystem.useLimitSwitch = !outtakeSystem.useLimitSwitch;
    }

    //Interface Methods
    public void update() {
        for (Subsystem s : subsystems) {
            s.update();
        }
    }

    public void toInit() {
        regularFall();
        for (Subsystem s : subsystems) {
            s.toInit();
        }
    }

}