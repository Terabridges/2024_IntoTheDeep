package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

@Config
@Autonomous(name="BucketAuto", group="Auto")
public class BucketAuto extends LinearOpMode
{
    StateMachine main;
    StateMachine score;
    StateMachine pickup;
    private ElapsedTime runtime = new ElapsedTime();

    //define poses

    //define PathChains

    enum mainStates
    {
        START,

        STOP
    }

    enum scoreStates
    {

    }

    enum pickupStates
    {

    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot r = new Robot(hardwareMap, telemetry);

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

    }
}
