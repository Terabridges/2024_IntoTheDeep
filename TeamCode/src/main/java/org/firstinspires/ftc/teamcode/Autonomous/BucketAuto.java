package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
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
    StateMachine dive;

    //Pedro
    private Follower follower;

    //Gamepad
    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    Pose startPose = new Pose(AConstants.BOT_CENTER_X, 108+ AConstants.BOT_CENTER_Y, Math.toRadians(0));

    Pose scorePoseP = new Pose(17.8, 135, Math.toRadians(335));
    Pose scorePose1 = new Pose(18.25, 135.5, Math.toRadians(350.5));
    Pose scorePose2 = new Pose(18.25, 135.5, Math.toRadians(349));
    Pose scorePose3 = new Pose(15.5, 131.5, Math.toRadians(315));
    Pose scorePose4 = new Pose(16.5, 131, Math.toRadians(315));

    Pose controlPointS = new Pose(32, 121, Math.toRadians(315));

    //Pose firstSampleStart = new Pose(21.5, 134.4, Math.toRadians(333.5));
    //Pose secondSampleStart = new Pose(19.3, 134.8, Math.toRadians(353));
    //Pose thirdSampleStart = new Pose(25, 130.5, Math.toRadians(30));
    //Pose firstSampleEnd = new Pose(AConstants.END_X, AConstants.FIRST_SAMPLE.getY(), Math.toRadians(0));
    //Pose secondSampleEnd = new Pose(AConstants.END_X, AConstants.SECOND_SAMPLE.getY(), Math.toRadians(0));
    //Pose thirdSampleEnd = new Pose(AConstants.END_X+4, AConstants.THIRD_SAMPLE.getY()+3, Math.toRadians(28.5));

    Pose firstSampleEnd = new Pose(21.5, 134.4, Math.toRadians(335));
    Pose secondSampleEnd = new Pose(20, 135, Math.toRadians(357.5));
    Pose thirdSampleEnd = new Pose(25, 130, Math.toRadians(32.5));

    Pose lane1 = new Pose(60, 93.25, Math.toRadians(270));
    Pose lane2 = new Pose(60, 93.25, Math.toRadians(270));
    Pose lane3 = new Pose(60, 93.25, Math.toRadians(270));
    Pose lane1C = new Pose(61, 109, Math.toRadians(270));
    Pose lane2C = new Pose(61, 109, Math.toRadians(270));
    Pose lane3C = new Pose(61 , 109, Math.toRadians(270));
    Pose[] lanes = {lane1, lane2, lane3};
    Pose[] lanesC = {lane1C, lane2C, lane3C};

    Pose placeHolder = new Pose(0, 0, Math.toRadians(0));
    Pose controlPoint = new Pose(28, 129, Math.toRadians(0));
    Pose controlPoint2 = new Pose(60, 98, Math.toRadians(270));
    Pose controlPoint3 = new Pose(68, 118, Math.toRadians(0));

    Pose controlPoint4 = new Pose(65, 128, Math.toRadians(0));

    Pose controlPointDive = new Pose(65, 128, Math.toRadians(0));
    Pose controlPointScore = new Pose(17.5, 126.5, Math.toRadians(315));


    //Pose[] samples = {firstSampleStart, secondSampleStart, thirdSampleStart};
    Pose[] scoreFrom = {placeHolder, firstSampleEnd, secondSampleEnd, thirdSampleEnd, placeHolder};
    Pose[] scoreTo = {scorePoseP, scorePose1, scorePose2, scorePose3, placeHolder};

    private PathChain goToScore, goToScoreControl, goToScoreFinal,
            intakeSample, goToSub2, goToSubC, thirdPickup, currentPose;

    //Enums
    enum mainStates
    {
        SCORE,
        PICKUP,
        DIVE,
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
        THIRD_PICKUP,
        MOVE_FORWARD,
        INTAKE_TRANSFER,
        SWIVEL_DOWN,
        OUTTAKE_TRANSFER,
        STOP
    }
    enum diveStates
    {
        GO_TO_SUB1,
        GO_TO_SUB2,
        SWEEP,
        EXTEND_INTAKE,
        EXTEND,
        DETECT_COLOR,
        SPIT,
        PARK,
        LIL_SPIT,
        RETRACT_INTAKE,
        SUCCESS,
        OUTTAKE_FALL,
        WAIT_STATE,
        OUTTAKE_RISE,
        SCORE,
        FLIP,
        OPEN,
        STOP
    }

    //Other
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime loopTime = new ElapsedTime();
    int curSample = 0;
    boolean failedPickup = false;
    boolean isPreload = true;
    boolean isRed = false;
    int selectedLane = 1;
    public double intakeTime = 1;

    @Override
    public void runOpMode() throws InterruptedException
    {
        r = new Robot(hardwareMap, telemetry);
        i = new IntakeSystem(hardwareMap);
        o = new OuttakeSystem(hardwareMap);
        v = new VisionSystem(hardwareMap);

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        o.manualOuttake = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        follower.setMaxPower(AConstants.STANDARD_POWER);

        buildPaths();
        buildStateMachines();

        while (opModeInInit()){
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad2.a && !previousGamepad2.a){
                isRed = !isRed;
            }

            if(currentGamepad2.b && !previousGamepad2.b){
                selectedLane++;
                if (selectedLane == lanes.length){ selectedLane = 0;}
            }

            telemetry.addData("Is Red", isRed);
            telemetry.addData("Lane", selectedLane+1);
            telemetry.update();
        }

        //Press Start
        waitForStart();

        o.resetSlideEncoders();

        runtime.reset();
        loopTime.reset();
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

        goToSubC = buildLinearPath(scorePose3, lanesC[selectedLane]);
        goToSub2 = buildLinearPath(lanesC[selectedLane], lanes[selectedLane]);

        //goToScoreControl = buildLinearPath(lanes[selectedLane], lanesC[selectedLane]);
        //goToScoreFinal = buildLinearPath(lanesC[selectedLane], scorePose4);

        //goToScoreControl = buildCurvedPath(lanes[selectedLane], controlPointDive, controlPointS);
        goToScoreControl = buildCurvedPath(lanes[selectedLane], controlPointDive, scorePose4);
        goToScoreFinal = buildLinearPath(controlPointScore, scorePose4);

        Pose currentP = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        currentPose = buildLinearPath(currentP, currentP);
    }

    public void buildStateMachines()
    {
        main = new StateMachineBuilder()
                .state(mainStates.SCORE)
                .onEnter(() -> score.start())
                .loop(() -> score.update())
                .transition(() -> curSample == 4 && score.getStateString().equals("STOP"), mainStates.DIVE)
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

                .state(mainStates.DIVE)
                .onEnter(() -> dive.start())
                .loop(() -> dive.update())
                //.transition(() -> dive.getStateString().equals("SUCCESS"), mainStates.SCORE)
                .transition(() -> dive.getStateString().equals("PARK_SUCCESS"), mainStates.STOP)

                .state(mainStates.STOP)

                .build();

        score = new StateMachineBuilder()
                .state(scoreStates.GO_TO_SCORE)
                .onEnter(() -> {
                    buildPaths();
                    //follower.followPath(goToScore, (((curSample == 0) || (curSample == 3)) ? AConstants.MID_POWER-.1 : AConstants.LOW_POWER), true);
                    follower.followPath(goToScore, ( curSample == 0 ? AConstants.MID_POWER+.1 : ( curSample == 3 ? AConstants.MID_POWER-.1 : AConstants.LOW_POWER) ), true);

                    o.outtakeSlidesHigh();
                    o.outtakeSwivelMid();
                    o.wristDown();
                    i.intakeSwivelRest();
                })
                .transition(() -> i.isSwivelRest(), scoreStates.EXTEND)

                .state(scoreStates.EXTEND)
                .onEnter(() -> {
                    if (curSample == 3) {

                    } else if (curSample == 2){
                        i.intakeSlidesSam();
                        i.intakeSwivelDown();
                    } else {
                        i.intakeSlidesExtend();
                        i.intakeSwivelDown();
                    }
                    i.intakeSlowSpinOut();
                })
                .transition(() -> o.isSlidesAlmostHigh() && !follower.isBusy(), scoreStates.DUNK)

                .state(scoreStates.DUNK)
                .onEnter(() -> {
                    o.outtakeSwivelUp();
                    o.wristUp();
                    i.intakeStopSpin();
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
                .state(pickupStates.THIRD_PICKUP)
                .onEnter(() -> {
                    o.outtakeSlidesRest();
                    if (curSample == 3)
                    {
                        buildPaths();
                        follower.followPath(intakeSample, AConstants.MID_POWER, true);
                    }
                })
                .transition(() -> !(curSample == 3), pickupStates.MOVE_FORWARD)
                .transition(() -> !follower.isBusy(), pickupStates.MOVE_FORWARD)

                .state(pickupStates.MOVE_FORWARD)
                .onEnter(() -> {
                    if (curSample != 3)
                    {
                        buildPaths();
                        follower.followPath(intakeSample, AConstants.MID_POWER-.05, true);
                    }
                    if (curSample == 3){
                        //i.intakeSlidesExtend();
                        i.intakeSlidesSuperExtend();
                    } else {
                        i.intakeSlidesSuperExtend();
                    }
                    i.intakeSpinIn();
                })
                .transitionTimed(intakeTime, pickupStates.INTAKE_TRANSFER)
                .onExit(() -> {
                    i.intakeStopSpin();
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
                    i.intakeSpinIn();
                })
                .transition( () -> (o.isSlidesDown() && o.isSwivelTransfer()), pickupStates.STOP, () -> {
                    o.closeClaw();
                    i.intakeStopSpin();
                })

                .state(pickupStates.STOP)

                .build();
        dive = new StateMachineBuilder()
                .state(diveStates.GO_TO_SUB1)
                .onEnter(() -> {
                    o.outtakeSlidesRest();
                    buildPaths();
                    follower.followPath(goToSubC, AConstants.STANDARD_POWER, true);
                })
                .transition(() -> !follower.isBusy(), diveStates.GO_TO_SUB2)

                .state(diveStates.GO_TO_SUB2)
                .onEnter(() -> {
                    buildPaths();
                    follower.followPath(goToSub2, AConstants.MID_POWER, true);
                })
                .transition(() -> !follower.isBusy(), diveStates.SWEEP)
                .transitionTimed(1, diveStates.SWEEP, () -> {
                    buildPaths();
                    follower.breakFollowing();
                })
                .onExit(() -> i.intakeSweeperOut())

                .state(diveStates.SWEEP)
                .onEnter(() -> i.intakeSwivelRest())
                .transitionTimed(0.4, diveStates.EXTEND_INTAKE)
                .onExit(() -> i.intakeSweeperIn())

                .state(diveStates.EXTEND_INTAKE)
                .onEnter(() -> {
                    i.intakeSlidesQuarter();
                    i.intakeSwivelDown();
                    i.intakeSpinIn();
                })
                .transition(() -> i.isIntakeQuarter(), diveStates.EXTEND)

                .state(diveStates.EXTEND)
                .onEnter(() -> i.intakeSlidesExtend())
                .transitionTimed(1, diveStates.LIL_SPIT, () -> {
                    i.intakeStopSpin();
                    i.setIntakeSlidesPIDF((int)i.intakeSlidesEnc.getCurrentPosition());
                })

                .state(diveStates.LIL_SPIT)
                .onEnter(() -> i.intakeSlowSpinOut())
                .transitionTimed(0.15, diveStates.DETECT_COLOR, () -> i.intakeStopSpin())


                .state(diveStates.DETECT_COLOR)
                .transition(() -> {
                    if (isRed){
                        return v.getColorValSensitive().equals("blue");
                    } else {
                        return v.getColorValSensitive().equals("red");
                    }
                }, diveStates.SPIT)
                .transition(() -> {
                    if (isRed){
                        return (v.getColorValSensitive().equals("red") || v.isColor("yellow"));
                    } else {
                        return (v.getColorValSensitive().equals("blue") || v.isColor("yellow"));
                    }
                }, diveStates.RETRACT_INTAKE)
                .transitionTimed(2.5, diveStates.PARK)

                .state(diveStates.SPIT)
                .onEnter(() -> i.intakeSpinOut())
                .transitionTimed(3, diveStates.PARK)

                .state(diveStates.RETRACT_INTAKE)
                .onEnter(() -> {
                    i.intakeSwivelRest();
                    i.intakeSlidesRetract();
                })
                .transition(() -> i.isIntakeRetracted(), diveStates.SUCCESS)

                .state(diveStates.SUCCESS)
                .onEnter(() -> {
                    o.outtakeSwivelTransfer();
                    o.wristTransfer();
                    i.intakeSpinIn();
                    i.intakeSwivelTransfer();
                })
                .transition(() -> i.isSwivelTransfer(), diveStates.OUTTAKE_FALL)

                .state(diveStates.OUTTAKE_FALL)
                .onEnter(() -> {
                    o.outtakeSlidesDown();
                    curSample++;
                    follower.followPath(goToScoreControl, AConstants.STANDARD_POWER, true);
                })
                .transition( () -> (o.isSlidesDown() && o.isSwivelTransfer()), diveStates.WAIT_STATE, () -> {
                    o.closeClaw();
                    i.intakeStopSpin();
                })

                .state(diveStates.WAIT_STATE)
                .transitionTimed(0.15, diveStates.OUTTAKE_RISE)

                .state(diveStates.OUTTAKE_RISE)
                .onEnter(() -> {
                    o.outtakeSlidesHigh();
                    o.outtakeSwivelMid();
                    o.wristDown();
                })
                .transition(() -> !follower.isBusy(), diveStates.SCORE)
                .transition(() -> runtime.seconds() >= 29.4, diveStates.FLIP, ()-> {
                    o.outtakeSwivelUp();
                    o.wristUp();
                })

                .state(diveStates.SCORE)
                .onEnter(() -> follower.followPath(goToScoreFinal, AConstants.LOW_POWER, true))
                .transition(() -> o.isSlidesHigh(), diveStates.FLIP)
                .transition(() -> runtime.seconds() >= 29, diveStates.FLIP)
                .onExit(() -> {
                    o.outtakeSwivelUp();
                    o.wristUp();
                })

                .state(diveStates.FLIP)
                .transition(() -> o.isSwivelUp(), diveStates.OPEN)
                .transition(() -> runtime.seconds() >= 29.5, diveStates.OPEN)

                .state(diveStates.OPEN)
                .onEnter(()->o.openClaw())
                .transitionTimed(0.2, diveStates.STOP, () -> o.outtakeSwivelDown())

                .state(diveStates.PARK)
                .onEnter(() -> {
                    i.intakeStopSpin();
                    o.outtakeSlidesPark();
                    o.outtakeSwivelPark();
                    o.closeClaw();
                    o.wristDown();
                })

                .state(diveStates.STOP)

                .build();
    }

    public void telemetry()
    {
        telemetry.addData("main state", main.getStateString());
        telemetry.addData("pickup state", pickup.getStateString());
        telemetry.addData("score state", score.getStateString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        //telemetry.addData("see something", v.isSomething());
        telemetry.addData("current sample", curSample);
        telemetry.addData("spinTarget", i.intakeSpinTarget);
        telemetry.addData("Outtake pos", o.outtakeBottomVertical.getCurrentPosition());
        telemetry.addData("Manual slides", o.manualOuttake);
        telemetry.addData("Mode", o.outtakeBottomVertical.getMode());
        telemetry.addData("Linear SLides POs", i.intakeSlidesEnc.getCurrentPosition());
        telemetry.addData("Swivel Pos", i.intakeSwivelEnc.getCurrentPosition());
        telemetry.addData("Loop Time", loopTime.milliseconds());
        telemetry.update();
        loopTime.reset();
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
