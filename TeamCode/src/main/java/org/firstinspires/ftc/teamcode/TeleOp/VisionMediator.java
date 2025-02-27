package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSystem;

public class VisionMediator {
    private VisionSystem vision;
    private DriveSystem drive;
    private Robot r;

    private boolean colliding = false;
    private double leftBackDist;
    private double rightBackDist;

    public VisionMediator(Robot r) {
        this.vision = r.visionSystem;
        this.drive = r.driveSystem;
        this.r = r;
    }

    public void colliding() {
        colliding = true;
    }

    public void notColliding() {
        colliding = false;
    }

    public void update() {
        leftBackDist = vision.leftBackDistVal;
        rightBackDist = vision.rightBackDistVal;

        //if drive.localier == where i want && vision.willstopat == true, set colliding to true, move to where i want, set colliding to false

    }
}
