package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.constants.PinpointConstants;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.Locale;


public class DriveSystem implements Subsystem {

    //Hardware
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    public GoBildaPinpointDriver odo;
    private Pose2D pos;
    public String data;

    //Software
    public boolean manualDrive = true;
    public boolean isIntakeExtended = false;
    public double fastTurn = 1;
    public double turnFactor = fastTurn;
    public double slowTurn = 0.4;
    public boolean useSlowMode = false;

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

        PinpointConstants.distanceUnit = DistanceUnit.INCH;

        odo.setOffsets(-3.5, 1);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();


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
