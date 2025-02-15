package org.firstinspires.ftc.teamcode.TeleOp.TeleControl;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSystem;

public class VisionControl implements Control {

    //Software
    VisionSystem vision;
    Gamepad gp1;
    Robot robot;

    //Constructor
    public VisionControl(VisionSystem vision, Gamepad gp1){
        this.vision = vision;
        this.gp1 = gp1;
    }

    public VisionControl(Robot robot, Gamepad gp1) {
        this(robot.visionSystem, gp1);
        this.robot = robot;
    }


    //Methods


    //Interface Methods
    @Override
    public void update(){

    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Current Color", vision.getColorVal());
    }

}
