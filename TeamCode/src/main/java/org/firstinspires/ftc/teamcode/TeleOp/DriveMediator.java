package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSystem;


public class DriveMediator {

    // PID
    public double pyaw = 0.015, iyaw = 0.0003, dyaw = 0.0001, fyaw = 0.0;
    public PIDFController yawPIDF = new PIDFController(pyaw, iyaw, dyaw, fyaw);

    public double paxial = 0.03, iaxial = 0.0005, daxial = 0.00005, faxial = 0.0;
    public PIDFController axialPIDF = new PIDFController(paxial, iaxial, daxial, faxial);

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

        yawPIDF.setPIDF(pyaw, iyaw, dyaw, fyaw);
        axialPIDF.setPIDF(paxial, iaxial, daxial, faxial);
    }

    // Methods
    public void colliding() {
        colliding = true;

        r.telemetry.addData("COLLIDING", "true");
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
        double robotBackHeading = robotHeading + 180;
        if (robotBackHeading >= 360) robotBackHeading -= 360;

        double headingDifference = Math.toRadians(fieldForwardHeading - robotHeading);
        double cosHeading = Math.cos(headingDifference);
        double sinHeading = Math.sin(headingDifference);

        double fieldAxial = axial * cosHeading - lateral * sinHeading;
        double fieldLateral = axial * sinHeading + lateral * cosHeading;

        r.telemetry.addData("Field Axial:", Double.toString(fieldAxial));
        r.telemetry.addData("Field Lateral:", Double.toString(fieldLateral));

        // Apply constraints to the drive vector
        if (fieldAxial > 0) {
            fieldAxial = 0;

            double yawErrorFront = AngleUnit.normalizeDegrees(fieldForwardHeading - robotHeading);
            double yawErrorBack = AngleUnit.normalizeDegrees(fieldForwardHeading - robotBackHeading);

            double yawError = Math.abs(yawErrorFront) < Math.abs(yawErrorBack) ? yawErrorFront : yawErrorBack;
            //double yawError = AngleUnit.normalizeDegrees(fieldForwardHeading - robotHeading);
            double rawYawPower = yawPIDF.calculate(yawError, 0);
            double yawPower = Math.max(-1, Math.min(1, rawYawPower));  // Clamping output

            yaw = yawPower;

            r.telemetry.addData("Yaw Power:" , Double.toString(yawPower));


        }

        // Use PIDF controller to adjust fieldAxial if the robot is ahead of the wall's line
        if (obstacle != null && obstacle.isColliding(pose)) {

            double positionError = 0;
            switch (obstacle.wallType) {
                case VERTICAL_LINE:
                case VERTICAL_LINE_INCH_THICK:
                    positionError = pose.getX(DistanceUnit.INCH) - obstacle.equation.getX(pose.getY(DistanceUnit.INCH));
                    break;
                case HORIZONTAL_LINE:
                case DIAGONAL_LINE:
                case DIAGONAL_LINE_ABOVE_COLLIDING:
                    positionError = pose.getY(DistanceUnit.INCH) - obstacle.equation.getY(pose.getX(DistanceUnit.INCH));
                    break;
            }

            if (positionError > 0) {
                double rawAxialPower = axialPIDF.calculate(positionError, 0);
                double axialPower = Math.max(-1, Math.min(0, rawAxialPower));  // Clamping output to negative values

                if (fieldAxial<0) {
                    fieldAxial = Math.min(fieldAxial, axialPower);
                }
                else {
                    fieldAxial = axialPower;
                }

                r.telemetry.addData("NEW Field Axial:", Double.toString(fieldAxial));
            }
        
            // Another Option: just make it so that if the robot is within some tolerance from the line, it is colliding, and then the robot needs to use the PIDF to square up against the true line if its colliding

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

    }
}



// Walls Enum
enum Walls {
    CHAMBERS(WallType.VERTICAL_LINE_INCH_THICK, 0, new Equation(0, 0, 42.5 - DriveSystem.BOT_CENTER_X), pose -> pose.getY(DistanceUnit.INCH) < 100 && pose.getY(DistanceUnit.INCH) > 44),
    //CHAMBERS(WallType.VERTICAL_LINE_INCH_THICK, 0, new Equation(0, 0, 42.5 - DriveSystem.BOT_CENTER_X)),
    NET_ZONE(WallType.DIAGONAL_LINE_ABOVE_COLLIDING, 135, new Equation(0, 1, 124 - DriveSystem.BOT_CENTER_X));


    public enum WallType {
        VERTICAL_LINE,
        HORIZONTAL_LINE,
        DIAGONAL_LINE,
        VERTICAL_LINE_INCH_THICK,
        DIAGONAL_LINE_ABOVE_COLLIDING;
    }

    public static class Equation {
        private double a=0, b=0, c=0;
        public Equation(double a, double b, double c) {
            this.a = a;
            this.b = b;
            this.c = c;
        }

        public double getX(double y) {
            return (a*y*y) + (b*y) + (c);
        }

        public double getY(double x) {
            return (a*x*x) + (b*x) + (c);
        }
    }

    @FunctionalInterface
    interface AdditionalCondition {
        boolean check(Pose2D pose);
    }

    private final double orthogonalHeading;
    public final WallType wallType;
    public final Equation equation;
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
                return Math.abs(x - equation.getX(y)) < 1.0; // Adjust threshold as needed
            case HORIZONTAL_LINE:
                return Math.abs(y - equation.getY(x)) < 1.0; // Adjust threshold as needed
            case DIAGONAL_LINE:
                return Math.abs(y - equation.getY(x)) < 1.0; // Adjust threshold as needed
            case VERTICAL_LINE_INCH_THICK:
                return (x - equation.getX(y)) > 0 && (x - equation.getX(y)) < 16.0; // Adjust threshold as needed
            case DIAGONAL_LINE_ABOVE_COLLIDING:
                return (y - equation.getY(x)) > 0; // Adjust threshold as needed
            default:
                return false;
        }
    }

}
