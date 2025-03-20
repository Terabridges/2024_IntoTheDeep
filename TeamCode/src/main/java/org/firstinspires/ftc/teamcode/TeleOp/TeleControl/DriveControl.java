package org.firstinspires.ftc.teamcode.TeleOp.TeleControl;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.TeleOp.DriveMediator;
import org.firstinspires.ftc.teamcode.Utility.EdgeDetector;

public class DriveControl implements Control {

    //Software
    DriveSystem driveSystem;
    public Robot robot;
    public Gamepad gp1;
    public Gamepad gp2;
    DriveMediator dM;
    public double FAST_MULT = 1.0;
    public double SLOW_MULT = 0.6;
    public double speed = FAST_MULT;
    EdgeDetector slowModeRE = new EdgeDetector( () -> toggleSlowMode());
    EdgeDetector virtualWallsRE = new EdgeDetector(() -> driveSystem.toggleWalls());

    public boolean noForward = false;

    //Constructor
    public DriveControl(DriveSystem d, Gamepad gp1, Gamepad gp2) {
        this.driveSystem = d;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public DriveControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.driveSystem, gp1, gp2);
        this.robot = robot;
        this.dM = robot.dM;
    }

    //Methods
    public void toggleSlowMode(){
        driveSystem.useSlowMode = !driveSystem.useSlowMode;
    }


    //Interface Methods
    @Override
    public void update(){

        dM.update();
        if (dM.isColliding()){
            noForward = true;
        }

        slowModeRE.update(gp1.x);
        virtualWallsRE.update(gp2.left_bumper);
        speed = (driveSystem.useSlowMode ? SLOW_MULT : FAST_MULT);

        if(driveSystem.manualDrive){

            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gp1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gp1.left_stick_x;
            double yaw = gp1.right_stick_x;

            if (dM.isColliding() && driveSystem.virtualWallOn) {
               double[] collidingVector = dM.calculateCollidingVector(axial, lateral, yaw);
               axial = collidingVector[0];
               lateral = collidingVector[1];
               yaw = collidingVector[2];
            }


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
        telemetry.addData("SPEED", (driveSystem.useSlowMode ? "SLOW" : "FAST"));

        telemetry.addData("Position", driveSystem.data);

        telemetry.addData("Virtual Walls", driveSystem.virtualWallOn);
    }

}
