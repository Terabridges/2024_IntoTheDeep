package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
@Autonomous(name="SpecimenAuto", group="Auto")
public class SpecimenAuto extends LinearOpMode
{
    //Subsystems
    Robot r;
    IntakeSystem i;
    OuttakeSystem o;
    //VisionSystem v;


    //Pedro
    private Follower follower;

    Pose startPose = new Pose(AConstants.BOT_CENTER_X, 48+ AConstants.BOT_CENTER_Y, Math.toRadians(180));

    Pose preloadPose = new Pose(32, 60, Math.toRadians(180));
    Pose preloadPoseb = new Pose(35.615, 60, Math.toRadians(180));
    Pose score1b = new Pose(35.615, 63, Math.toRadians(180));
    Pose score2b = new Pose(35.615, 68, Math.toRadians(180));
    Pose score3b = new Pose(35.615, 73, Math.toRadians(180));
    Pose score1 = new Pose(32, 63, Math.toRadians(180));
    Pose scoreControl1 = new Pose(24, 60, Math.toRadians(180));
    Pose score2 = new Pose(32, 68, Math.toRadians(180));
    Pose scoreControl2 = new Pose(24, 65, Math.toRadians(180));
    Pose score3 = new Pose(32, 73, Math.toRadians(180));
    Pose scoreControl3 = new Pose(24, 70, Math.toRadians(180));

    Pose start1 = new Pose(60, 25.5, Math.toRadians(90));
    Pose end1 = new Pose(20, 25.5, Math.toRadians(90));
    Pose start2 = new Pose(55, 18.5, Math.toRadians(90));
    Pose end2 = new Pose(20, 18.5, Math.toRadians(90));
    Pose start3 = new Pose(50, 7, Math.toRadians(90));
    Pose end3 = new Pose(20, 7, Math.toRadians(90));

    Pose control1 = new Pose(24, 49, Math.toRadians(90));
    Pose control2 = new Pose(50, 35, Math.toRadians(90));
    Pose control3 = new Pose(50, 25, Math.toRadians(90));

    Pose prep = new Pose(20, 24, Math.toRadians(0));
    Pose pick = new Pose(11.65, 24, Math.toRadians(0));
    Pose pickb = new Pose(6, 24, Math.toRadians(0));

    private PathChain scorePreload, scorePreloadb, goPick, goPickb, goPrep1, goPrep2, goPrep3, goScore1, goScore2, goScore3, goScore1b, goScore2b, goScore3b;
    private PathChain pushSamples1, pushSamples2, pushSamples3, pushSamples4, pushSamples5, pushSamples6,
            goPark;

    Pose[] score = {preloadPoseb, score1b, score2b, score3b};
    Pose[] push = {start1, end1, start2, end2, start3, end3};
    Pose[] control = {control1, control2, control3};

    PathChain[] goPrep = {goPrep1, goPrep2, goPrep3};
    PathChain[] goScore = {goScore1, goScore2, goScore3};

    //State Factory
    StateMachine main;
    StateMachine scoreSpec;
    StateMachine grab;
    StateMachine pushSpec;

    //Enums
    enum mainStates
    {
        SCORE,
        GRAB,
        PUSH,
        STOP
    }
    enum scoreStates
    {
        GO_TO_SCORE1,
        GO_TO_SCORE2,
        CLIP,
        STOP
    }
    enum grabStates
    {
        GO_PREP,
        ADVANCE1,
        ADVANCE2,
        STOP
    }
    enum pushStates
    {
        PUSH1,
        PUSH2,
        PUSH3,
        PUSH4,
        PUSH5,
        PUSH6,
        STOP
    }

    //Other
//    private PoseUpdater poseUpdater;
//    private DashboardPoseTracker dashboardPoseTracker;

    public ElapsedTime runtime = new ElapsedTime();
    int curSpec = 0;
    boolean isPreload = true;
    boolean endCheck = false;
    double pickPower = 0.5;

    @Override
    public void runOpMode() throws InterruptedException
    {
        r = new Robot(hardwareMap, telemetry);
        i = new IntakeSystem(hardwareMap);
        o = new OuttakeSystem(hardwareMap);
        //v = new VisionSystem(hardwareMap);

        o.manualOuttake = false;

//        poseUpdater = new PoseUpdater(hardwareMap);
//        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
//        Drawing.sendPacket();

        FConstants.setTValue(0.98);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        follower.setMaxPower(AConstants.STANDARD_POWER);

        buildPaths();

        buildStateMachines();

        //Press Start
        waitForStart();

        o.resetSlideEncoders();

        runtime.reset();
        r.toInit();
        main.start();

        //Main Loop
        while (opModeIsActive())
        {
            follower.update();
            main.update();
            r.update();

//            poseUpdater.update();
            telemetry();
//            Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
//            Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
//            Drawing.sendPacket();

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
        scorePreload = buildLinearPath(startPose, preloadPoseb);
        scorePreloadb = buildLinearPath(preloadPose, preloadPoseb);

        goPick = buildLinearPath(prep, pickb);
        goPickb = buildLinearPath(pick, pickb);
        goPrep1 = buildLinearPath(push[3], prep);
        //goScore1 = buildLinearPath(pick, score1b);
        goScore1 = buildCurvedPath(pick, scoreControl1, score1b);
        goScore1b = buildLinearPath(score1, score1b);
        goPrep2 = buildLinearPath(score1, prep);
        //goScore2 = buildLinearPath(pick, score2b);
        goScore2 = buildCurvedPath(pick, scoreControl2, score2b);
        goScore2b = buildLinearPath(score2, score2b);
        goPrep3 = buildLinearPath(score2, prep);
        goPark = buildLinearPath(score2, pick);
        //goScore3 = buildLinearPath(pick, score3b);
        goScore3 = buildCurvedPath(pick, scoreControl3, score3b);
        goScore3b = buildLinearPath(score3, score3b);

        pushSamples1 = buildCurvedPath(score[0], control[0], push[0]);
        pushSamples2 = buildLinearPath(push[0], push[1]);
        pushSamples3 = buildCurvedPath(push[1], control[1], push[2]);
        pushSamples4 = buildLinearPath(push[2], push[3]);
        pushSamples5 = buildCurvedPath(push[3], control[2], push[4]);
        pushSamples6 = buildLinearPath(push[4], push[5]);

//        pushSamples = follower.pathBuilder()
//                .addPath(bezierCurve(score[0], control[0], push[0]))
//                .setLinearHeadingInterpolation(score[0].getHeading(), push[0].getHeading())
//
//                .addPath(bezierLine(push[0], push[1]))
//                .setLinearHeadingInterpolation(push[0].getHeading(), push[1].getHeading())
//                .addPath(bezierCurve(push[1], control[1], push[2]))
//                .setLinearHeadingInterpolation(push[1].getHeading(), push[2].getHeading())
//
//                .addPath(bezierLine(push[2], push[3]))
//                .setLinearHeadingInterpolation(push[2].getHeading(), push[3].getHeading())
//                .addPath(bezierCurve(push[3], control[2], push[4]))
//                .setLinearHeadingInterpolation(push[3].getHeading(), push[4].getHeading())
//
//                .addPath(bezierLine(push[4], push[5]))
//                .setLinearHeadingInterpolation(push[4].getHeading(), push[5].getHeading())
//
//                .build();
    }

    public void buildStateMachines()
    {
        main = new StateMachineBuilder()
                .state(mainStates.SCORE)
                .onEnter(() -> scoreSpec.start())
                .loop(() -> scoreSpec.update())
                .transition(() -> scoreSpec.getStateString().equals("STOP") && isPreload, mainStates.PUSH)
                .transition(() -> scoreSpec.getStateString().equals("STOP"), mainStates.GRAB)
                .onExit(() -> {
                    if (isPreload)
                        isPreload = false;
                    else
                        curSpec++;
                    scoreSpec.reset();
                })

                .state(mainStates.GRAB)
                .onEnter(() -> grab.start())
                .loop(() -> grab.update())
                .transition(() -> grab.getStateString().equals("STOP") && endCheck, mainStates.STOP)
                .transition(() -> grab.getStateString().equals("STOP"), mainStates.SCORE)
                .onExit(() -> {
                    //follower.setMaxPower(AConstants.A_LOW);
                    grab.reset();
                })

                .state(mainStates.PUSH)
                .onEnter(() -> {
                    pushSpec.start();
                    //follower.setMaxPower(AConstants.STANDARD_POWER);
                })
                .loop(() -> pushSpec.update())
                .transition(() -> pushSpec.getStateString().equals("STOP"), mainStates.GRAB)
                .onExit(() -> {
                    pushSpec.reset();
                    //follower.setMaxPower(AConstants.A_LOW);
                })

                .state(mainStates.STOP)

                .build();

        scoreSpec = new StateMachineBuilder()
                .state(scoreStates.GO_TO_SCORE1)
                .onEnter(() -> {
                    //follower.setMaxPower(AConstants.MID_POWER);
                    //runtime.reset();
                    if (isPreload)
                        follower.followPath(scorePreload, AConstants.STANDARD_POWER, true);
                    else
                        //follower.followPath(goScore[curSpec], true);
                        if (curSpec == 0)
                            follower.followPath(goScore1, AConstants.STANDARD_POWER, true);
                        else if (curSpec == 1)
                            follower.followPath(goScore2, AConstants.STANDARD_POWER, true);
                        else if (curSpec == 2)
                            follower.followPath(goScore3, AConstants.STANDARD_POWER, true);
                    o.outtakeSlidesScore1();
                    o.wristLock();
                    o.outtakeSwivelLock();
                })
                .transition(() -> !follower.isBusy() && o.isSlidesScore1(), scoreStates.CLIP)

                .state(scoreStates.GO_TO_SCORE2)
                .onEnter(() ->
                {
                    runtime.reset();
                    if (isPreload)
                        follower.followPath(scorePreloadb, true);
                    else
                        //follower.followPath(goScore[curSpec], true);
                        if (curSpec == 0)
                            follower.followPath(goScore1b, true);
                        else if (curSpec == 1)
                            follower.followPath(goScore2b, true);
                        else if (curSpec == 2)
                            follower.followPath(goScore3b, true);
                })
                .transition(() -> !follower.isBusy() && runtime.seconds() > .4, scoreStates.CLIP)

                .state(scoreStates.CLIP)
                .onEnter(() -> {
                    o.outtakeSlidesScore2();
                })
                .transitionTimed(0.3, scoreStates.STOP)
                .onExit(() -> {
                    o.openClaw();
                    o.outtakeSwivelDown();
                })

                .state(scoreStates.STOP)

                .build();

        grab = new StateMachineBuilder()
                .state(grabStates.GO_PREP)
                .onEnter(() -> {
                    if (curSpec < 3)
                    {
                        //follower.followPath(goPrep[curSpec], true);
                        if (curSpec == 0)
                            follower.followPath(goPrep1, AConstants.MID_POWER, true);
                        else if (curSpec == 1)
                            follower.followPath(goPrep2, AConstants.MID_POWER, true);
                        else if (curSpec == 2)
                            follower.followPath(goPark, AConstants.MID_POWER, true); //Was goPrep3
                        o.wristGrab();
                        o.outtakeSwivelGrab();
                        o.openClaw();
                        i.intakeSwivelRest();
                    }
                    else
                        endCheck = true;
                })
                .transition(() -> !follower.isBusy() && i.isSwivelRest() && endCheck, grabStates.STOP)
                .transition(() -> !follower.isBusy() && i.isSwivelRest(), grabStates.ADVANCE1)
                .onExit(() -> {
                    //follower.setMaxPower(pickPower-.05);
                    o.outtakeSlidesGrab1();
                })

                .state(grabStates.ADVANCE1)
                .onEnter(() -> follower.followPath(goPick, AConstants.MID_POWER,true))
                .transition(() -> !follower.isBusy(), grabStates.STOP)
                .onExit(() -> {
                    o.closeClaw();
                    //follower.setMaxPower(AConstants.A_LOW);
                })

                .state(grabStates.ADVANCE2)
                .onEnter(() -> follower.followPath(goPickb,true))
                .transition(() -> !follower.isBusy(), grabStates.STOP)

                .state(grabStates.STOP)

                .build();

        pushSpec = new StateMachineBuilder()
                .state(pushStates.PUSH1)
                .onEnter(() -> {
                    o.outtakeSlidesRest();
                    o.outtakeSwivelDown();
                    o.wristDown();
                    follower.followPath(pushSamples1, AConstants.STANDARD_POWER, true);
                })
                .transition(() -> !follower.isBusy(), pushStates.PUSH2)

                .state(pushStates.PUSH2)
                .onEnter(() -> follower.followPath(pushSamples2, AConstants.STANDARD_POWER, true))
                .transition(() -> !follower.isBusy(), pushStates.PUSH3)

                .state(pushStates.PUSH3)
                .onEnter(() -> follower.followPath(pushSamples3, AConstants.STANDARD_POWER, true))
                .transition(() -> !follower.isBusy(), pushStates.PUSH4)

                .state(pushStates.PUSH4)
                .onEnter(() -> follower.followPath(pushSamples4, AConstants.STANDARD_POWER, true))
                .transition(() -> !follower.isBusy(), pushStates.STOP)

                .state(pushStates.PUSH5)
                .onEnter(() -> follower.followPath(pushSamples5, AConstants.STANDARD_POWER, true))
                .transition(() -> !follower.isBusy(), pushStates.PUSH6)

                .state(pushStates.PUSH6)
                .onEnter(() -> follower.followPath(pushSamples6, AConstants.STANDARD_POWER, true))
                .transition(() -> !follower.isBusy(), pushStates.STOP)

                .state(pushStates.STOP)
                //.onEnter(() -> follower.setMaxPower(AConstants.A_LOW))

                .build();
    }

    public void telemetry()
    {
        telemetry.addData("main state", main.getStateString());
        telemetry.addData("grab state", grab.getStateString());
        telemetry.addData("score state", scoreSpec.getStateString());
        telemetry.addData("push state", pushSpec.getStateString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        //telemetry.addData("see something", v.isSomething());
        telemetry.addData("current specimen", curSpec);
        telemetry.addData("spinTarget", i.intakeSpinTarget);
        telemetry.addData("Outtake pos", o.outtakeBottomVertical.getCurrentPosition());
        telemetry.addData("Manual slides", o.manualOuttake);
        telemetry.addData("Mode", o.outtakeBottomVertical.getMode());
        telemetry.update();
    }

    private PathChain buildLinearPath(Pose start, Pose end) {
        return new PathBuilder()
                .addPath(bezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }
    private BezierLine bezierLine(Pose start, Pose end)
    {
        return new BezierLine(new Point(start), new Point(end));
    }

    private PathChain buildCurvedPath(Pose start, Pose control, Pose end) {
        return new PathBuilder()
                .addPath(bezierCurve(start, control, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }
    private BezierCurve bezierCurve(Pose start, Pose control, Pose end)
    {
        return new BezierCurve(new Point(start), new Point(control), new Point(end));
    }
}
