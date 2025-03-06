package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSystem;

public class DriveMediator {
    private VisionSystem vision;
    private DriveSystem drive;
    private Robot r;

    private boolean colliding = false;
    private double leftBackDist;
    private double rightBackDist;

    private Pose2D pose;

    private double fieldForwardHeading;

    public DriveMediator(Robot r) {
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

    public boolean isColliding() {
        return colliding;
    }

    public void setFieldForwardHeading(double fieldForwardHeading) {
        this.fieldForwardHeading = fieldForwardHeading;
    }

    public double getFieldForwardHeading() {
        return leftBackDist;
    }

    public double getRobotHeading() {
        return pose.getHeading(AngleUnit.DEGREES);
    }

    public void update() {
        leftBackDist = vision.leftBackDistVal;
        rightBackDist = vision.rightBackDistVal;

        pose = drive.pos;

        if (pose.getX(DistanceUnit.INCH) > 42.5 + drive.BOT_CENTER_X) {
            colliding();
            setFieldForwardHeading(0);
        }

        //if drive.localier == where i want && vision.willstopat == true, set colliding to true, move to where i want, set colliding to false

    }
}
