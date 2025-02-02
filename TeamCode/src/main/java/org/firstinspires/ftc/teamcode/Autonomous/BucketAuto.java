package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.pathgen.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

@Config
@Autonomous(name="BucketAuto", group="Auto")
public class BucketAuto extends LinearOpMode
{
    //Subsystems
    Robot r;
    IntakeSystem i;
    OuttakeSystem o;
    VisionSystem v;

    //State Factory
    StateMachine main;
    StateMachine score;
    StateMachine pickup;

    //Pedro
    private Follower follower;

    Pose startPose = new Pose(AConstants.BOT_CENTER_X, 96+ AConstants.BOT_CENTER_Y, Math.toRadians(0));
    Pose scorePose = new Pose(24-2, 120+2, Math.toRadians(315));

    Pose firstSampleStart;
    Pose secondSampleStart;
    Pose thirdSampleStart;
    Pose firstSampleEnd;
    Pose secondSampleEnd;
    Pose thirdSampleEnd;


    Pose[] samples = {firstSampleStart, secondSampleStart, thirdSampleStart};
    Pose[] scoreFrom = {startPose, firstSampleEnd, secondSampleEnd, thirdSampleEnd};

    private PathChain goToScore, goToSample, intakeSample, failedIntake;

    //Enums
    enum mainStates
    {
        SCORE,
        PICKUP,
        STOP
    }

    enum scoreStates
    {
        GO_TO_SCORE,
        //RAISE_OUTTAKE, (If you want to split up movement + raising into two.)
        DUNK,
        OPEN_CLAW,
        CLOSE_CLAW,
        LOWER_OUTTAKE,
        STOP
    }

    enum pickupStates
    {
        END_CHECK,
        GO_TO_PICKUP,
        EXTEND_INTAKE,
        RETRACT_INTAKE,
        TRANSFER,
        FAILED_PICKUP,
        STOP

    }

    //Other
    public ElapsedTime runtime = new ElapsedTime();
    int curSample = 0;
    boolean failedPickup = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        r = new Robot(hardwareMap, telemetry);
        i = new IntakeSystem(hardwareMap);
        o = new OuttakeSystem(hardwareMap);
        v = new VisionSystem(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        r.toInit();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
        buildStateMachines();

        //Press Start
        waitForStart();
        runtime.reset();

        main.start();

        //Main Loop
        while (opModeIsActive())
        {
            r.update();
            follower.update();
            main.update();

            telemetry();

            if (main.getStateString().equals("STOP"))
            {
                main.stop();
                main.reset();
                return;
            }
        }
    }

    public void buildPaths()
    {
        goToScore =
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(scoreFrom[curSample]),
                                        new Point(scorePose)))
                        .setLinearHeadingInterpolation(scoreFrom[curSample].getHeading(), scorePose.getHeading())
                        .build();
        goToSample =
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(scorePose),
                                        new Point(samples[curSample-1])))
                        .setLinearHeadingInterpolation(scorePose.getHeading(), samples[curSample-1].getHeading())
                        .build();
        intakeSample =
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(samples[curSample-1]),
                                        new Point(scoreFrom[curSample])))
                        .setLinearHeadingInterpolation(samples[curSample-1].getHeading(), scoreFrom[curSample].getHeading())
                        .build();
        failedIntake =
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(scoreFrom[curSample]),
                                        new Point(samples[curSample])))
                        .setLinearHeadingInterpolation(scoreFrom[curSample].getHeading(), samples[curSample].getHeading())
                        .build();
    }

    public void buildStateMachines()
    {
        main = new StateMachineBuilder()
                .state(mainStates.SCORE)
                .onEnter(() -> score.start())
                .loop(() -> score.update())
                .transition(() -> score.getStateString().equals("STOP"), mainStates.PICKUP)
                .onExit(() -> {
                    score.reset();
                    curSample++;
                })

                .state(mainStates.PICKUP)
                .onEnter(() -> pickup.start())
                .loop(() -> pickup.update())
                .transition(() -> curSample >= 4, mainStates.STOP) //TD: Park here
                .transition(() -> pickup.getStateString().equals("STOP") && !failedPickup, mainStates.SCORE)
                .transition(() -> pickup.getStateString().equals("STOP") && failedPickup, mainStates.PICKUP)
                .onExit(() -> {
                    pickup.reset();
                    failedPickup = false;
                })

                .state(mainStates.STOP)

                .build();

        score = new StateMachineBuilder()
                .state(scoreStates.GO_TO_SCORE)
                .onEnter(() -> {
                    buildPaths();
                    follower.followPath(goToScore);
                    o.outtakeSlidesHigh();
                })
                .transition(() -> o.isSlidesHigh() && !follower.isBusy(), scoreStates.DUNK)

                .state(scoreStates.DUNK)
                .onEnter(() -> o.outtakeSwivelUp())
                .transition(() -> o.isSwivelUp(), scoreStates.OPEN_CLAW)

                .state(scoreStates.OPEN_CLAW)
                .onEnter(() -> {
                    o.openClaw();
                    runtime.reset();
                })
                .transition(() -> runtime.seconds() > AConstants.DROP_TIME, scoreStates.CLOSE_CLAW)

                .state(scoreStates.CLOSE_CLAW)
                .onEnter(() -> {
                    o.closeClaw();
                    o.outtakeSwivelDown();
                })
                .transition(() -> o.isSwivelDown(), scoreStates.LOWER_OUTTAKE)

                .state(scoreStates.LOWER_OUTTAKE)
                .onEnter(() -> o.outtakeSlidesDown())
                .transition(() -> o.isSlidesDown(), scoreStates.STOP)

                .state(scoreStates.STOP)

                .build();

        pickup = new StateMachineBuilder()
                .state(pickupStates.END_CHECK)
                .transition(() -> curSample >= 4, pickupStates.STOP)
                .transition(() -> curSample < 4, pickupStates.GO_TO_PICKUP)

                .state(pickupStates.GO_TO_PICKUP)
                .onEnter(() -> {
                    buildPaths();
                    follower.followPath(goToSample);
                })
                .transition(() -> !follower.isBusy(), pickupStates.EXTEND_INTAKE)
                .onExit(() -> i.intakeSlidesExtend())

                .state(pickupStates.EXTEND_INTAKE)
                .onEnter(() -> {
                    i.intakeSpinIn();
                    buildPaths();
                    follower.followPath(intakeSample);
                })
                .transition(() -> v.isSomething() && !follower.isBusy(), pickupStates.RETRACT_INTAKE)
                .transition(() -> !v.isSomething() && !follower.isBusy(), pickupStates.FAILED_PICKUP)
                .onExit(() -> i.intakeStopSpin())

                .state(pickupStates.RETRACT_INTAKE)
                .onEnter(() -> i.intakeSlidesRetract())
                .transition(() -> i.isIntakeRetracted(), pickupStates.TRANSFER)

                .state(pickupStates.TRANSFER)
                //.onEnter(() -> TRANSFER HERE JOSH)
                //.transition(() -> TRANSFER DONE, pickupStates.STOP)

                .state(pickupStates.FAILED_PICKUP)
                .onEnter(() ->
                {
                    if (curSample < 3)
                    {
                        failedPickup = true;
                        buildPaths();
                        follower.followPath(failedIntake);
                    }
                })
                .transition(() -> !follower.isBusy(), pickupStates.STOP)
                .onExit(() -> curSample++)

                .state(pickupStates.STOP)

                .build();
    }

    public void telemetry()
    {
        telemetry.addData("main state", main.getStateString());
        telemetry.addData("pickup state", pickup.getStateString());
        telemetry.addData("score state", score.getStateString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("see something", v.isSomething());
        telemetry.update();
    }
}
