package org.firstinspires.ftc.teamcode.TeleOp.TeleControl;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSystem;
import org.firstinspires.ftc.teamcode.TeleOp.DriveMediator;
import org.firstinspires.ftc.teamcode.Utility.EdgeDetector;

public class VisionControl implements Control {

    //Software
    VisionSystem vision;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    DriveMediator dM;
    EdgeDetector visionMode = new EdgeDetector( () -> vision.switchVisionMode());

    //Constructor
    public VisionControl(VisionSystem vision, Gamepad gp1, Gamepad gp2){
        this.vision = vision;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public VisionControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.visionSystem, gp1, gp2);
        this.robot = robot;
        this.dM = robot.dM;
    }


    //Methods
    public void handleCollisions(double leftBackDistVal, double rightBackDistVal) {
        if ((leftBackDistVal + rightBackDistVal) / 2 < 60) {
            dM.colliding();
        }
    }

    //Interface Methods
    @Override
    public void update(){

        visionMode.update(gp2.y);

        if (vision.willStopAtObstacle) {
            handleCollisions(vision.leftBackDistVal, vision.rightBackDistVal);
        }

    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Current Color", vision.getColorVal());
        telemetry.addData("Left Back Dist", vision.leftBackDistVal);
        telemetry.addData("Right Back Dist", vision.rightBackDistVal);
        telemetry.addData("Vision Mode", (vision.specimenVisionMode ? "specimen" : "sample"));
    }

}
