package org.firstinspires.ftc.teamcode.Autonomous.MainAuto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.Autonomous.Util.AConstants;
import org.firstinspires.ftc.teamcode.Autonomous.Util.Paths.SpecimenPaths;
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
    SpecimenPaths s;

    //Pedro
    private Follower follower;

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
        WAIT_STATE,
        FLIP,
        GO_TO_SCORE2,
        WAIT,
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
    double grabResetTime = 1;

    @Override
    public void runOpMode() throws InterruptedException
    {
        r = new Robot(hardwareMap, telemetry);
        i = new IntakeSystem(hardwareMap);
        o = new OuttakeSystem(hardwareMap);
        //v = new VisionSystem(hardwareMap);
        s = new SpecimenPaths();

        o.manualOuttake = false;

//        poseUpdater = new PoseUpdater(hardwareMap);
//        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
//        Drawing.sendPacket();

        FConstants.setTValue(.975);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(s.getStartPose());
        follower.setMaxPower(AConstants.STANDARD_POWER);

        s.buildPathsSpecimen();

        buildStateMachines();

        //Press Start
        waitForStart();

        o.resetSlideEncoders();

        runtime.reset();
        r.toInit();
        o.outtakeSlidesRest();
        o.wristAuto();
        o.outtakeSwivelAuto();

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
                    runtime.reset();
                    if (isPreload)
                        follower.followPath(s.scorePreload, AConstants.STANDARD_POWER, true);
                    else
                        follower.followPath(s.getScore(curSpec), AConstants.STANDARD_POWER, true);
                    //o.outtakeSlidesRest();
                })
                .transitionTimed(0, scoreStates.WAIT_STATE)

                .state(scoreStates.WAIT_STATE)
                .onEnter(() -> {
                    o.outtakeSlidesAutoScore1();
                    o.wristAutoScore();
                    o.outtakeSwivelAutoScore();
                })
                .transition(() -> !follower.isBusy(), scoreStates.GO_TO_SCORE2)

                .state(scoreStates.GO_TO_SCORE2)
                .onEnter(() ->
                {
                    runtime.reset();
                    if (isPreload)
                        follower.followPath(s.scorePreloadb, AConstants.MID_POWER, true);
                    else
                        follower.followPath(s.getScoreB(curSpec), AConstants.STANDARD_POWER, true);
                })
                .transition(() -> !follower.isBusy(), scoreStates.WAIT)

                .state(scoreStates.WAIT)
                .transitionTimed(0.1, scoreStates.CLIP)

                .state(scoreStates.CLIP)
                .onEnter(() -> {
                    o.outtakeSlidesAutoScore2();
                })
                //.transitionTimed(0.3, scoreStates.STOP)
                .transition(() -> o.isSlidesAutoScore2(), scoreStates.STOP)
                .onExit(() -> {
                    o.openClaw();
                    o.outtakeSwivelTransfer();
                })

                .state(scoreStates.STOP)

                .build();

        grab = new StateMachineBuilder()
                .state(grabStates.GO_PREP)
                .onEnter(() -> {
                    if (curSpec < 3)
                    {
                        follower.followPath(s.getPrep(curSpec), AConstants.STANDARD_POWER, true);
                        o.wristAuto();
                        o.outtakeSwivelAuto();
                        o.openClaw();
                    }
                    else
                        endCheck = true;
                })
                .transition(() -> !follower.isBusy() && endCheck, grabStates.STOP)
                .transition(() -> !follower.isBusy(), grabStates.ADVANCE1)
                .onExit(() -> {
                    //follower.setMaxPower(pickPower-.05);
                    o.outtakeSlidesDown();
                })

                .state(grabStates.ADVANCE1)
                .onEnter(() -> follower.followPath(s.goPick,  AConstants.MID_POWER,true))
                .transition(() -> !follower.isBusy(), grabStates.STOP)
                .transitionTimed(grabResetTime, grabStates.STOP, () -> follower.breakFollowing())
                .onExit(() -> o.closeClaw())

                .state(grabStates.ADVANCE2)
                .onEnter(() -> follower.followPath(s.goPickb,  AConstants.MID_POWER,true))
                .transition(() -> !follower.isBusy(), grabStates.STOP)
                //.onExit(() -> o.closeClaw())

                .state(grabStates.STOP)

                .build();

        pushSpec = new StateMachineBuilder()
                .state(pushStates.PUSH1)
                .onEnter(() -> {
                    o.outtakeSlidesRest();
                    o.outtakeSwivelDown();
                    o.wristTransfer();
                    follower.followPath(s.pushSamples1, AConstants.STANDARD_POWER, true);
                })
                .transition(() -> !follower.isBusy(), pushStates.PUSH2)

                .state(pushStates.PUSH2)
                .onEnter(() -> follower.followPath(s.pushSamples2, AConstants.STANDARD_POWER, true))
                .transition(() -> !follower.isBusy(), pushStates.PUSH3)

                .state(pushStates.PUSH3)
                .onEnter(() -> follower.followPath(s.pushSamples3, AConstants.STANDARD_POWER, true))
                .transition(() -> !follower.isBusy(), pushStates.PUSH4)

                .state(pushStates.PUSH4)
                .onEnter(() -> follower.followPath(s.pushSamples4, AConstants.STANDARD_POWER, true))
                .transition(() -> !follower.isBusy(), pushStates.STOP)

                .state(pushStates.PUSH5)
                .onEnter(() -> follower.followPath(s.pushSamples5, AConstants.STANDARD_POWER, true))
                .transition(() -> !follower.isBusy(), pushStates.PUSH6)

                .state(pushStates.PUSH6)
                .onEnter(() -> follower.followPath(s.pushSamples6, AConstants.STANDARD_POWER, true))
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
        //telemetry.addData("spinTarget", i.intakeSpinTarget);
        telemetry.addData("Outtake pos", o.outtakeMiddleVertical.getCurrentPosition());
        //telemetry.addData("Manual slides", o.manualOuttake);
        //telemetry.addData("Mode", o.outtakeBottomVertical.getMode());
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
