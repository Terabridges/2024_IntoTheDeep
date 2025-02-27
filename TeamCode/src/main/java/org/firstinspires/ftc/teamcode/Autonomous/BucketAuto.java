package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSystem;
import org.firstinspires.ftc.teamcode.TeleOp.MainTeleOp;
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
    //VisionSystem v;

    //State Factory
    StateMachine main;
    StateMachine score;
    StateMachine pickup;

    //Pedro
    private Follower follower;

    Pose startPose = new Pose(AConstants.BOT_CENTER_X, 108+ AConstants.BOT_CENTER_Y, Math.toRadians(0));
    Pose scorePoseP = new Pose(15, 133, Math.toRadians(336.5));
    Pose scorePose1 = new Pose(17.5, 135, Math.toRadians(349));
    Pose scorePose2 = new Pose(17.5, 135, Math.toRadians(349));
    Pose scorePose3 = new Pose(15, 129, Math.toRadians(315));

    //Pose firstSampleStart = new Pose(AConstants.START_X, AConstants.FIRST_SAMPLE.getY(), Math.toRadians(0));
    //Pose secondSampleStart = new Pose(AConstants.START_X, AConstants.SECOND_SAMPLE.getY(), Math.toRadians(0));
    //Pose thirdSampleStart = new Pose(AConstants.START_X, AConstants.THIRD_SAMPLE.getY(), Math.toRadians(31));
    //Pose firstSampleEnd = new Pose(AConstants.END_X, AConstants.FIRST_SAMPLE.getY(), Math.toRadians(0));
    //Pose secondSampleEnd = new Pose(AConstants.END_X, AConstants.SECOND_SAMPLE.getY(), Math.toRadians(0));
    //Pose thirdSampleEnd = new Pose(AConstants.END_X+4, AConstants.THIRD_SAMPLE.getY()+3, Math.toRadians(28.5));

    Pose firstSampleEnd = new Pose(22, 131.5, Math.toRadians(336.5));
    Pose secondSampleEnd = new Pose(21, 134, Math.toRadians(353));
    Pose thirdSampleEnd = new Pose(25, 129, Math.toRadians(40));

    Pose placeHolder = new Pose(0, 0, Math.toRadians(0));
    Pose controlPoint = new Pose(28, 129, Math.toRadians(28.5));

    //Pose[] samples = {firstSampleStart, secondSampleStart, thirdSampleStart};
    Pose[] scoreFrom = {placeHolder, firstSampleEnd, secondSampleEnd, thirdSampleEnd};
    Pose[] scoreTo = {scorePoseP, scorePose1, scorePose2, scorePose3};

    private PathChain goToScore, goToScorePreload, goToSample, intakeSample, curvedIntake, failedIntake;

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
        EXTEND,
        DUNK,
        OPEN_CLAW,
        STOP
    }

    enum pickupStates
    {
        MOVE_FORWARD,
        INTAKE_TRANSFER,
        SWIVEL_DOWN,
        OUTTAKE_TRANSFER,
        STOP
    }

    //Other
    public ElapsedTime runtime = new ElapsedTime();
    int curSample = 0;
    boolean failedPickup = false;
    boolean isPreload = true;

    @Override
    public void runOpMode() throws InterruptedException
    {
        r = new Robot(hardwareMap, telemetry);
        i = new IntakeSystem(hardwareMap);
        o = new OuttakeSystem(hardwareMap);

        o.manualOuttake = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

            telemetry();

            i.intakeSetSpin(i.intakeSpinTarget);

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
        goToScore = buildLinearPath(scoreFrom[curSample], scoreTo[curSample]);
        //goToSample = buildLinearPath(scoreTo[curSample], samples[curSample]);
        if (curSample > 0)
            intakeSample = buildLinearPath(scoreTo[curSample-1], scoreFrom[curSample]);
        else
            intakeSample = buildLinearPath(scoreTo[curSample], scoreFrom[curSample]);
        //failedIntake = buildLinearPath(scoreFrom[curSample], samples[curSample]);
        curvedIntake = buildCurvedPath(scorePose2, controlPoint, thirdSampleEnd);
    }

    public void buildStateMachines()
    {
        main = new StateMachineBuilder()
                .state(mainStates.SCORE)
                .onEnter(() -> score.start())
                .loop(() -> score.update())
                .transition(() -> curSample > 3 && score.getStateString().equals("STOP"), mainStates.STOP) //TD: Park here
                .transition(() -> score.getStateString().equals("STOP"), mainStates.PICKUP)
                .onExit(() -> score.reset())

                .state(mainStates.PICKUP)
                .onEnter(() -> pickup.start())
                .loop(() -> pickup.update())
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
                    follower.setMaxPower(AConstants.LOW_POWER);
                    buildPaths();
                    follower.followPath(goToScore, true);
                    o.outtakeSlidesHigh();
                    o.outtakeSwivelMid();
                    i.intakeSwivelRest();
                })
                .transition(() -> i.isSwivelRest(), scoreStates.EXTEND)

                .state(scoreStates.EXTEND)
                .onEnter(() -> {
                    if (curSample == 2) {
                        i.intakeSlidesHalf();
                        i.intakeSwivelDown();
                    }
                    else if (curSample == 3) {
                    }
                    else {
                        i.intakeSlidesExtend();
                        i.intakeSwivelDown();
                    }
                })
                .transition(() -> o.isSlidesHigh() && !follower.isBusy(), scoreStates.DUNK)

                .state(scoreStates.DUNK)
                .onEnter(() -> {
                    follower.setMaxPower(AConstants.STANDARD_POWER);
                    o.outtakeSwivelUp();
                    o.wristUp();
                })
                .transition(() -> o.isSwivelUp(), scoreStates.OPEN_CLAW)

                .state(scoreStates.OPEN_CLAW)
                .onEnter(() -> o.openClaw())
                .transitionTimed(AConstants.DROP_TIME, scoreStates.STOP)
                .onExit(() -> {
                    o.outtakeSwivelTransfer();
                    o.wristTransfer();
                    curSample++;
                })

                .state(scoreStates.STOP)

                .build();

        pickup = new StateMachineBuilder()
                .state(pickupStates.MOVE_FORWARD)
                .onEnter(() -> {
                    buildPaths();
                    o.outtakeSlidesRest();
                    i.intakeSpinTarget = -1;
//                    if (curSample == 3)
//                        follower.followPath(curvedIntake, true);
//                    else
                        follower.followPath(intakeSample, true);
                })
                .transition(() -> !follower.isBusy(), pickupStates.INTAKE_TRANSFER)
                .onExit(() -> {
                    i.intakeSpinTarget = 0;
                    i.intakeSwivelRest();
                })

                //TRANSFER
                .state(pickupStates.INTAKE_TRANSFER)
                .onEnter( () -> {
                    i.intakeSlidesRetract();
                    o.openClaw();
                })
                .transition(() -> i.isIntakeRetracted(), pickupStates.SWIVEL_DOWN)

                .state(pickupStates.SWIVEL_DOWN)
                .onEnter(() -> {
                    i.intakeSwivelTransfer();
                })
                .transition(() -> i.isSwivelTransfer(), pickupStates.OUTTAKE_TRANSFER)

                .state(pickupStates.OUTTAKE_TRANSFER)
                .onEnter(() -> {
                    o.outtakeSwivelTransfer();
                    o.outtakeSlidesDown();
                    o.wristTransfer();
                })
                .transition( () -> o.isSlidesDown(), pickupStates.STOP, () -> o.closeClaw())

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
        //telemetry.addData("see something", v.isSomething());
        telemetry.addData("current sample", curSample);
        telemetry.addData("spinTarget", i.intakeSpinTarget);
        telemetry.addData("Outtake pos", o.outtakeBottomVertical.getCurrentPosition());
        telemetry.addData("Manual slides", o.manualOuttake);
        telemetry.addData("Mode", o.outtakeBottomVertical.getMode());
        telemetry.addData("Linear SLides POs", i.intakeSlidesEnc.getCurrentPosition());
        telemetry.addData("Swivel Pos", i.intakeSwivelEnc.getCurrentPosition());
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
