package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


public class DriveSystem implements Subsystem {

    //Hardware
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    //Software
    public boolean manualDrive = true;
    public boolean isIntakeExtended = false;
    public double fastTurn = 1;
    public double turnFactor = fastTurn;
    public double slowTurn = 0.4;

    //Constructor
    public DriveSystem(HardwareMap map) {
        leftBack = map.get(DcMotor.class, "left_back");
        leftFront = map.get(DcMotor.class, "left_front");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack = map.get(DcMotor.class, "right_back");
        rightFront = map.get(DcMotor.class, "right_front");
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
    }

    @Override
    public void update(){
        if (isIntakeExtended){
            turnFactor = slowTurn;
        } else {
            turnFactor = fastTurn;
        }
    }

}
