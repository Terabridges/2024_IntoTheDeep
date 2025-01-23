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

    //State Factory
    StateMachine main;
    StateMachine score;
    StateMachine pickup;

    //Pedro
    private Follower follower;
    public Pose startPose = new Pose();
    public Pose scorePose = new Pose();

    public PathChain from0, from1, from2, from3;

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
        DUNK,
        OPEN_CLAW,
        CLOSE_CLAW,
        LOWER_OUTTAKE,
        STOP
    }

    enum pickupStates
    {

    }

    //Other
    private ElapsedTime runtime = new ElapsedTime();
    int curSample = 0;
    double dropTime = 0.2;

    @Override
    public void runOpMode() throws InterruptedException
    {
        r = new Robot(hardwareMap, telemetry);
        i = new IntakeSystem(hardwareMap);
        o = new OuttakeSystem(hardwareMap);

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

            /*
            if (main.getStateEnum() == main.STOP)
            {
                main.stop();
                main.reset();
                return;
            }
            */
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
                .loop( () -> score.update())
                .transition(() -> score.getStateString().equals("STOP"))
                .onExit(() -> score.reset())

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
                .transition(() -> o.isSlidesHigh() &&
                        (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)))

                .state(scoreStates.DUNK)
                .onEnter(() -> {
                    o.outtakeSwivelUp();
                })
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
                .onEnter(() -> {
                    o.outtakeSlidesDown();
                })
                .transition(() -> o.isSlidesDown())

                .state(scoreStates.STOP)

                .build();
    }
}
