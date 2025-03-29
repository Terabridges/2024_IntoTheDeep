package org.firstinspires.ftc.teamcode.Tests;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Autonomous.Util.AConstants;

@Disabled
@TeleOp(name="NetZoneAlignTest", group="Test")
@Config

public class NetZoneAlignTest extends LinearOpMode {

    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;
    public AnalogInput leftBackDistance;
    public AnalogInput rightBackDistance;
    private Follower follower;

    private Pose startPose = new Pose(AConstants.BOT_CENTER_X, 96+ AConstants.BOT_CENTER_Y, Math.toRadians(0));
    private Pose scorePose = new Pose(24-6, 120+6, Math.toRadians(315));
    private Pose currentPose;



    @Override
    public void runOpMode() {
        leftBack = hardwareMap.get(DcMotor .class, "left_back");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDistance = hardwareMap.get(AnalogInput.class, "left_back_distance");
        rightBackDistance = hardwareMap.get(AnalogInput.class, "right_back_distance");
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            currentPose = follower.getPose();

            if (currentPose.getX() < 20 && currentPose.getY() > 124 && currentPose.getHeading() < 140 && currentPose.getHeading() > 130) {

                if (gamepad1.left_stick_x <= 0.2 && gamepad1.left_stick_y <= 0.2 && gamepad1.right_stick_x <= 0.2) {
                    Pose alignpose = new Pose(currentPose.getX(), currentPose.getY(), Math.toRadians(180));

                    follower.followPath(buildToScorePath(currentPose));
                }

            }

        }
    }

    public PathChain buildToScorePath(Pose pose) {
        PathChain goToScore =
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(pose),
                                        new Point(scorePose)))
                        .setLinearHeadingInterpolation(pose.getHeading(), scorePose.getHeading())
                        .build();
        return goToScore;
    }
}
