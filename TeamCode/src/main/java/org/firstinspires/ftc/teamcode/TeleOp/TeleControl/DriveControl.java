package org.firstinspires.ftc.teamcode.TeleOp.TeleControl;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utility.EdgeDetector;

public class DriveControl implements Control {

    //Software
    DriveSystem driveSystem;
    public Robot robot;
    public Gamepad gp1;
    public double speed = 1.0;
    public double SLOW_MULT = 0.6;
    boolean useSlowMode = false;
    boolean manualDrive = true;
    EdgeDetector slowModeRE = new EdgeDetector( () -> toggleSlowMode());

    //Constructor
    public DriveControl(DriveSystem d, Gamepad gp1) {
        this.driveSystem = d;
        this.gp1 = gp1;
    }

    public DriveControl(Robot robot, Gamepad gp1) {
        this(robot.driveSystem, gp1);
        this.robot = robot;
    }

    //Methods
    public void toggleSlowMode(){
        useSlowMode = !useSlowMode;
    }


    //Interface Methods
    @Override
    public void update(){

        slowModeRE.update(gp1.right_bumper);
        speed = (useSlowMode) ? SLOW_MULT : 1.0;

        if(manualDrive){
            driveSystem.drive.setTeleOpMovementVectors(speed * (-gp1.left_stick_y), speed * (-gp1.left_stick_x), speed * (-gp1.right_stick_x));
        }
    }

    @Override
    public void addTelemetry(Telemetry telemetry){

    }

}
