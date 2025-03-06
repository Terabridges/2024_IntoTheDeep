package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSystem;

public class DriveMediator {

    // PID
    public double pyaw = 0.003, iyaw = 0.005, dyaw = 0.00005;
    public PIDController yawPID = new PIDController(pyaw, iyaw, dyaw);


    // Instance Variables
    private VisionSystem vision;
    private DriveSystem drive;
    private Robot r;

    private boolean colliding = false;
    private double leftBackDist;
    private double rightBackDist;

    public Pose2D pose;

    private double fieldForwardHeading;

    private Walls obstacle;

    // Constructors
    public DriveMediator(Robot r) {
        this.vision = r.visionSystem;
        this.drive = r.driveSystem;
        this.r = r;

        yawPID.setPID(pyaw, iyaw, dyaw);
    }

    // Methods
    public void colliding() {
        colliding = true;
    }

    public void colliding(Walls wall) {
        colliding();
        obstacle = wall;
        setFieldForwardHeading(wall.getOrthogonalHeading());
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
        return fieldForwardHeading;
    }

    public double getRobotHeading() {
        return pose.getHeading(AngleUnit.DEGREES);
    }



    public double[] calculateCollidingVector(double axial, double lateral, double yaw) {
        // Transform the drive vector to the field's coordinate system
        double fieldForwardHeading = getFieldForwardHeading();
        double robotHeading = getRobotHeading();
        double headingDifference = Math.toRadians(fieldForwardHeading - robotHeading);
        double cosHeading = Math.cos(headingDifference);
        double sinHeading = Math.sin(headingDifference);


        double fieldAxial = axial * cosHeading - lateral * sinHeading;
        double fieldLateral = axial * sinHeading + lateral * cosHeading;

        // Apply constraints to the drive vector
        if (fieldAxial > 0) {
            fieldAxial = 0;

            yaw = yawPID.calculate(robotHeading, fieldForwardHeading);


        }

        // Transform the constrained drive vector back to the robot's coordinate system
        axial = fieldAxial * cosHeading + fieldLateral * sinHeading;
        lateral = -fieldAxial * sinHeading + fieldLateral * cosHeading;

        return new double[] {axial, lateral, yaw};

    }


    // Update
    public void update() {
        leftBackDist = vision.leftBackDistVal;
        rightBackDist = vision.rightBackDistVal;

        pose = drive.pos;

        for (Walls wall : Walls.values()) {
            if (wall.isColliding(pose)) {
                colliding(wall);
                break;
            } else {
                notColliding();
            }
        }

        //if drive.localier == where i want && vision.willstopat == true, set colliding to true, move to where i want, set colliding to false

    }
}



// Walls Enum
enum Walls {
    CHAMBERS(WallType.VERTICAL_LINE_INCH_THICK, 0, new Equation(0, 0, 42.5), pose -> pose.getY(DistanceUnit.INCH) < 86.5 && pose.getY(DistanceUnit.INCH) > 57.5),
    NET_ZONE(WallType.DIAGONAL_LINE_ABOVE_COLLIDING, 135, new Equation(0, 1, 120));


    private enum WallType {
        VERTICAL_LINE,
        HORIZONTAL_LINE,
        DIAGONAL_LINE,
        VERTICAL_LINE_INCH_THICK,
        DIAGONAL_LINE_ABOVE_COLLIDING;
    }

    private static class Equation {
        private double a=0, b=0, c=0;
        private Equation(double a, double b, double c) {
            this.a = a;
            this.b = b;
            this.c = c;
        }

        private double getY(double y) {
            return (a*y*y) + (b*y) + (c);
        }

        private double getX(double x) {
            return (a*x*x) + (b*x) + (c);
        }
    }

    @FunctionalInterface
    interface AdditionalCondition {
        boolean check(Pose2D pose);
    }

    private final double orthogonalHeading;
    private final WallType wallType;
    private final Equation equation;
    private final AdditionalCondition additionalCondition;

    private Walls(WallType wallType, double orthogonalHeading, Equation equation) {
        this(wallType, orthogonalHeading, equation, pose -> true); // Default additional condition is always true
    }

    private Walls(WallType wallType, double orthogonalHeading, Equation equation,  AdditionalCondition additionalCondition) {
        this.orthogonalHeading = orthogonalHeading;
        this.wallType = wallType;
        this.equation = equation;
        this.additionalCondition = additionalCondition;
    }

    public double getOrthogonalHeading() {
        return orthogonalHeading;
    }

    public boolean isColliding(Pose2D pose) {
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);

        if (!additionalCondition.check(pose)) {
            return false;
        }

        switch (wallType) {
            case VERTICAL_LINE:
                return Math.abs(x - equation.getX(x)) < 1.0; // Adjust threshold as needed
            case HORIZONTAL_LINE:
                return Math.abs(y - equation.getY(y)) < 1.0; // Adjust threshold as needed
            case DIAGONAL_LINE:
                return Math.abs(y - equation.getY(x)) < 1.0; // Adjust threshold as needed
            case VERTICAL_LINE_INCH_THICK:
                return (x - equation.getX(x)) > 0 && (x - equation.getX(x)) < 1.0; // Adjust threshold as needed
            case DIAGONAL_LINE_ABOVE_COLLIDING:
                return (y - equation.getY(x)) > 0; // Adjust threshold as needed
            default:
                return false;
        }
    }

}
