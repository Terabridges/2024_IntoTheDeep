package org.firstinspires.ftc.teamcode.TeleOp.TeleControl;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSystem;
import org.firstinspires.ftc.teamcode.TeleOp.VisionMediator;
import org.firstinspires.ftc.teamcode.Utility.EdgeDetector;

public class VisionControl implements Control {

    //Software
    VisionSystem vision;
    Gamepad gp1;
    Robot robot;
    VisionMediator vM;
    EdgeDetector stopAtObs = new EdgeDetector( () -> vision.switchWillStop() );

    //Constructor
    public VisionControl(VisionSystem vision, Gamepad gp1){
        this.vision = vision;
        this.gp1 = gp1;
    }

    public VisionControl(Robot robot, Gamepad gp1) {
        this(robot.visionSystem, gp1);
        this.robot = robot;
        this.vM = robot.vM;
    }


    //Methods
    public void handleCollisions(double leftBackDistVal, double rightBackDistVal) {
        if ((leftBackDistVal + rightBackDistVal) / 2 < 60) {
            vM.colliding();
        }
    }

    //Interface Methods
    @Override
    public void update(){

        stopAtObs.update(gp1.dpad_up);

        if (vision.willStopAtObstacle) {
            handleCollisions(vision.leftBackDistVal, vision.rightBackDistVal);
        }

        vM.update();

    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Current Color", vision.getColorVal());
        telemetry.addData("Left Back Distance", vision.leftBackDistVal);
        telemetry.addData("Right Back Distance", vision.rightBackDistVal);
        telemetry.addData("Will Stop?", vision.willStopAtObstacle);
    }

}
