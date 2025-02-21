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
    public double FAST_MULT = 1.0;
    public double SLOW_MULT = 0.6;
    public double speed = FAST_MULT;
    boolean useSlowMode = false;
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

        slowModeRE.update(gp1.x);
        speed = (useSlowMode ? SLOW_MULT : FAST_MULT);

        if(driveSystem.manualDrive){
            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gp1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gp1.left_stick_x;
            double yaw = gp1.right_stick_x;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + (yaw * driveSystem.turnFactor);
            double rightFrontPower = axial - lateral - (yaw * driveSystem.turnFactor);
            double leftBackPower = axial - lateral + (yaw * driveSystem.turnFactor);
            double rightBackPower = axial + lateral - (yaw * driveSystem.turnFactor);
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
            leftFrontPower *= speed;
            rightFrontPower *= speed;
            leftBackPower *= speed;
            rightBackPower *= speed;
            driveSystem.leftFront.setPower(leftFrontPower);
            driveSystem.rightFront.setPower(rightFrontPower);
            driveSystem.leftBack.setPower(leftBackPower);
            driveSystem.rightBack.setPower(rightBackPower);
        }
    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("SPEED", (useSlowMode ? "SLOW" : "FAST"));
    }

}
