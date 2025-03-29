package org.firstinspires.ftc.teamcode.Autonomous.MainAuto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Util.Paths.BucketPaths;
import org.firstinspires.ftc.teamcode.Autonomous.Util.AConstants;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import com.pedropathing.follower.Follower;
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
    BucketPaths b;

    //State Factory
    StateMachine main;
    StateMachine score;
    StateMachine pickup;
    StateMachine dive;

    //Gamepad
    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    //Pedro
    private Follower follower;

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
        CLIP_WAIT,
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
    boolean isRed = false;
    int selectedLane = 1;
    public double intakeTime = 1;
    double subResetTime = 1;

    @Override
    public void runOpMode() throws InterruptedException
    {
        r = new Robot(hardwareMap, telemetry);
        i = new IntakeSystem(hardwareMap);
        o = new OuttakeSystem(hardwareMap);
        v = new VisionSystem(hardwareMap);
        b = new BucketPaths();

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        o.manualOuttake = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FConstants.setTValue(0.95);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(b.getStartPose());

        follower.setMaxPower(AConstants.STANDARD_POWER);

        b.buildPathsBucket(curSample, selectedLane);
        buildStateMachines();

        while (opModeInInit()){
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad2.a && !previousGamepad2.a){
                isRed = !isRed;
            }

            if(currentGamepad2.b && !previousGamepad2.b){
                selectedLane++;
                if (selectedLane == b.getLanesLength()){ selectedLane = 0;}
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
                .transition(() -> dive.getStateString().equals("PARK_SUCCESS"), mainStates.STOP)

                .state(mainStates.STOP)

                .build();

        score = new StateMachineBuilder()
                .state(scoreStates.GO_TO_SCORE)
                .onEnter(() -> {
                    b.buildPathsBucket(curSample, selectedLane);
                    follower.followPath(b.goToScore, ( curSample == 0 ? AConstants.MID_POWER+.1 : ( curSample == 3 ? AConstants.MID_POWER-.1 : AConstants.LOW_POWER) ), true);

                    o.outtakeSlidesHigh();
                    o.outtakeSwivelMid();
                    o.wristDown();
                    i.intakeSwivelRest();
                })
                .transition(() -> i.isSwivelRest(), scoreStates.EXTEND)

                .state(scoreStates.EXTEND)
                .onEnter(() -> {
                    if (curSample == 2){
                        i.intakeSlidesSam();
                        i.intakeSwivelDown();
                    } else if (curSample != 3) {
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
                    if (curSample == 3)
                    {
                        b.buildPathsBucket(curSample, selectedLane);
                        follower.followPath(b.intakeSample, AConstants.MID_POWER, true);
                        o.outtakeSlidesRest();
                    }
                })
                .transition(() -> !(curSample == 3), pickupStates.MOVE_FORWARD)
                .transition(() -> !follower.isBusy(), pickupStates.MOVE_FORWARD)

                .state(pickupStates.MOVE_FORWARD)
                .onEnter(() -> {
                    if (curSample != 3)
                    {
                        b.buildPathsBucket(curSample, selectedLane);
                        follower.followPath(b.intakeSample, AConstants.MID_POWER-.05, true);
                    }
                    if (curSample == 3){
                        i.intakeSlidesSuperExtend();
                    } else {
                        i.intakeSlidesSuperExtend();
                    }
                    i.intakeSpinIn();
                })
                .transitionTimed(0.1, pickupStates.CLIP_WAIT)

                .state(pickupStates.CLIP_WAIT)
                .onEnter(() -> o.outtakeSlidesRest())
                .transitionTimed(intakeTime - 0.1, pickupStates.INTAKE_TRANSFER)
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
                .onEnter(() -> i.intakeSwivelTransfer())
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
                    b.buildPathsBucket(curSample, selectedLane);
                    follower.followPath(b.goToSub1, AConstants.STANDARD_POWER, true);
                })
                .transition(() -> !follower.isBusy(), diveStates.GO_TO_SUB2)

                .state(diveStates.GO_TO_SUB2)
                .onEnter(() -> {
                    b.buildPathsBucket(curSample, selectedLane);
                    follower.followPath(b.goToSub2, AConstants.MID_POWER, true);
                })
                .transition(() -> !follower.isBusy(), diveStates.SWEEP)
                .transitionTimed(subResetTime, diveStates.SWEEP, () -> {
                    b.buildPathsBucket(curSample, selectedLane);
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
                .transitionTimed(1.4, diveStates.LIL_SPIT, () -> {
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
                    follower.followPath(b.goToScoreFinal, AConstants.STANDARD_POWER, true);
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
}
