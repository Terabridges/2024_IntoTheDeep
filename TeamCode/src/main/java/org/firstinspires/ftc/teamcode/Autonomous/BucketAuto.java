package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSystem;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;

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
    public Pose startPose = new Pose();
    public Pose scorePose = new Pose();

    public PathChain from0, from1, from2, from3, pickup0, pickup1, pickup2, pickup3;

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
        GO_TO_PICKUP,
        EXTEND_INTAKE,
        RETRACT_INTAKE,
        TRANSFER,
        FAILED_PICKUP,
        STOP

    }

    //Other
    private ElapsedTime runtime = new ElapsedTime();
    int curSample = 0;
    double dropTime = 0.2;
    double pickupTime = 0.8;

    @Override
    public void runOpMode() throws InterruptedException
    {
        r = new Robot(hardwareMap, telemetry);
        i = new IntakeSystem(hardwareMap);
        o = new OuttakeSystem(hardwareMap);
        v = new VisionSystem(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        r.toInit();

        buildPaths();
        buildStateMachines();

        waitForStart();
        runtime.reset();

        main.start();

        while (opModeIsActive())
        {
            r.update();
            telemetry.update();

            main.update();

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

    }

    public void buildStateMachines()
    {
        StateMachine main = new StateMachineBuilder()
                .state(mainStates.SCORE)
                .onEnter(() -> score.start())
                .loop(() -> score.update())
                .transition(() -> curSample == 4, mainStates.STOP)
                .transition(() -> score.getStateString().equals("STOP"), mainStates.PICKUP)
                .onExit(() -> {
                    score.reset();
                    curSample++;
                })

                .state(mainStates.PICKUP)
                .onEnter(() -> pickup.start())
                .loop(() -> pickup.update())
                .transition(() -> pickup.getStateString().equals("STOP"), mainStates.SCORE)
                .onExit(() -> pickup.reset())

                .build();

        StateMachine score = new StateMachineBuilder()
                .state(scoreStates.GO_TO_SCORE)
                .onEnter(() -> {
                    if (curSample == 0){follower.followPath(from0);}
                    else if (curSample == 1){follower.followPath(from1);}
                    else if (curSample == 2){follower.followPath(from2);}
                    else if (curSample == 3){follower.followPath(from3);}
                    o.outtakeSlidesHigh();
                })
                .transition(() -> o.isSlidesHigh() && !follower.isBusy())

                .state(scoreStates.DUNK)
                .onEnter(() -> o.outtakeSwivelUp())
                .transition(() -> o.isSwivelUp())

                .state(scoreStates.OPEN_CLAW)
                .onEnter(() -> {
                    o.openClaw();
                    runtime.reset();
                })
                .transition(() -> runtime.seconds() > dropTime)

                .state(scoreStates.CLOSE_CLAW)
                .onEnter(() -> {
                    o.closeClaw();
                    o.outtakeSwivelDown();
                })
                .transition(() -> o.isSwivelDown())

                .state(scoreStates.LOWER_OUTTAKE)
                .onEnter(() -> o.outtakeSlidesDown())
                .transition(() -> o.isSlidesDown())

                .state(scoreStates.STOP)

                .build();

        StateMachine pickup = new StateMachineBuilder()
                .state(pickupStates.GO_TO_PICKUP)
                .onEnter(() -> {
                    if (curSample == 0){follower.followPath(pickup0);}
                    else if (curSample == 1){follower.followPath(pickup1);}
                    else if (curSample == 2){follower.followPath(pickup2);}
                    else if (curSample == 3){follower.followPath(pickup3);}
                })
                .transition(() -> !follower.isBusy())
                .onExit(() -> i.intakeSlidesExtend())

                .state(pickupStates.EXTEND_INTAKE)
                .onEnter(() -> {
                    i.intakeSpinIn();
                    //Slowly drive forward
                })
                .transition(() -> v.isSomething())
                .transition(() -> runtime.seconds() > pickupTime, pickupStates.FAILED_PICKUP)
                .onExit(() -> i.intakeStopSpin())

                .state(pickupStates.RETRACT_INTAKE)
                .onEnter(() -> i.intakeSlidesRetract())
                .transition(() -> i.isIntakeRetracted())

                .state(pickupStates.TRANSFER)
                //.onEnter(() -> TRANSFER HERE JOSH)
                //.transition(() -> TRANSFER DONE)

                .state(pickupStates.STOP)

                .build();
    }
}
