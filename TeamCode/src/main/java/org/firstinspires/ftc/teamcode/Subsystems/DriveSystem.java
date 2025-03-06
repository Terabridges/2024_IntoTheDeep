package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.localization.GoBildaPinpointDriver;
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

    public Pose2D pos;
    public String data;
    public final Pose2D START_POS = new Pose2D(DistanceUnit.INCH, BOT_CENTER_X, 0, AngleUnit.DEGREES, 0);
    public final double INCH_TO_MM = 25.4;
    public static final double ROBOT_WIDTH = 13.117; //Front to back
    public static final double ROBOT_HEIGHT = 13.413; //Side to side
    public static final double BOT_CENTER_X = ROBOT_WIDTH /2;
    public static final double BOT_CENTER_Y = ROBOT_HEIGHT /2;

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

    public void driveStop(){
        manualDrive = true;
    }

    //Interface Methods
    @Override
    public void toInit(){

        odo.setOffsets((-3.5 * INCH_TO_MM), (1 * INCH_TO_MM));
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.setPosition(START_POS);
        pos = odo.getPosition();


    }

    @Override
    public void update(){
        if (isIntakeExtended){
            turnFactor = slowTurn;
        } else {
            turnFactor = fastTurn;
        }

        odo.update();
        pos = odo.getPosition();
        data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));

    }

}
