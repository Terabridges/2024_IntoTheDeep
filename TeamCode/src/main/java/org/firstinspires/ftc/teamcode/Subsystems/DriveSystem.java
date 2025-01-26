package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


public class DriveSystem implements Subsystem {

    //Hardware
    public Follower drive;

    //Software

    //Constructor
    public DriveSystem(HardwareMap map, Pose startPos) {
        drive = new Follower(map);
        drive.setStartingPose(startPos);
        Constants.setConstants(FConstants.class, LConstants.class);
    }

    public DriveSystem(HardwareMap map) {
        this(map, new Pose(0, 0, 0));
    }

    //Methods
    public void setPose(Pose pose){
        drive.setPose(pose);
    }

    public void setPower(Pose powers){
        drive.setTeleOpMovementVectors(powers.getX(), powers.getY(), powers.getHeading());
    }

    public Follower getDrive() {
        return drive;
    }

    //Interface Methods
    @Override
    public void toInit(){
        drive.startTeleopDrive();
    }

    @Override
    public void update(){
        drive.update();
    }

}
