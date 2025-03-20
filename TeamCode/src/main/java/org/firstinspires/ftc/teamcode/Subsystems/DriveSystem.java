package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


import java.util.Locale;


public class DriveSystem implements Subsystem {

    //Hardware
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    public GoBildaPinpointDriver odo;

    //Software
    public boolean manualDrive = true;
    public boolean isIntakeExtended = false;
    public double fastTurn = 1;
    public double turnFactor = fastTurn;
    public double slowTurn = 0.4;
    public boolean useSlowMode = false;
    public boolean virtualWallOn = false;

    public Pose2D pos;
    public String data;
    public final double INCH_TO_MM = 25.4;
    public static final double ROBOT_WIDTH = 13.117; //Front to back
    public static final double ROBOT_HEIGHT = 13.413; //Side to side
    public static final double BOT_CENTER_X = ROBOT_WIDTH /2;
    public static final double BOT_CENTER_Y = ROBOT_HEIGHT /2;

    public final Pose2D START_POS = new Pose2D(DistanceUnit.MM, BOT_CENTER_X*INCH_TO_MM, BOT_CENTER_Y*INCH_TO_MM, AngleUnit.RADIANS, 0);

    //Constructor
    public DriveSystem(HardwareMap map) {
        leftBack = map.get(DcMotor.class, "left_back");
        leftFront = map.get(DcMotor.class, "left_front");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack = map.get(DcMotor.class, "right_back");
        rightFront = map.get(DcMotor.class, "right_front");

        odo = map.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    //Methods
    public void driveBack(){
        manualDrive = false;
        leftBack.setPower(0.6);
        leftFront.setPower(0.6);
        rightBack.setPower(0.6);
        rightFront.setPower(0.6);
    }

    public void toggleWalls(){
        virtualWallOn = !virtualWallOn;
    }

    public void driveStop(){
        manualDrive = true;
    }

    public Pose2D updatePose() {

        Pose2D robotcentric = odo.getPosition();

        double newX = START_POS.getX(DistanceUnit.MM) + robotcentric.getX(DistanceUnit.MM);
        double newY = START_POS.getY(DistanceUnit.MM) + robotcentric.getY(DistanceUnit.MM);
        double newHead = START_POS.getHeading(AngleUnit.DEGREES) + robotcentric.getHeading(AngleUnit.DEGREES);

        Pose2D newpos = new Pose2D(DistanceUnit.MM, newX, newY, AngleUnit.DEGREES, newHead);

        return newpos;

    }

    //Interface Methods
    @Override
    public void toInit(){

        odo.setOffsets((-3.5 * INCH_TO_MM), (1 * INCH_TO_MM));
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.update();
        pos = updatePose();


    }

    @Override
    public void update(){
        if (isIntakeExtended){
            turnFactor = slowTurn;
        } else {
            turnFactor = fastTurn;
        }

        odo.update();
        pos = updatePose();
        data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));

    }

}
