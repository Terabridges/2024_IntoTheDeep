package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.TeleOp.TeleControl.*;
import org.firstinspires.ftc.teamcode.Utility.*;

import com.sfdev.assembly.state.*;
import java.util.HashMap;

@Disabled
@TeleOp(name="TeleOp", group="TeleOp")
public class MainTeleOp extends LinearOpMode {

    public HashMap<String, String> gamepadMap = null;
    private ElapsedTime runtime = new ElapsedTime();

    //Global FSM
    enum globalStates {
        INIT,
        INTAKE,
        TRANSFER,
        OUTTAKE
    }

    StateMachine intakeMachine;
    StateMachine transferMachine;
    StateMachine outtakeMachine;

    //Run OpMode
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, telemetry, gamepad1);

        intakeMachine = StateMachines.getIntakeMachine(robot);
        transferMachine = StateMachines.getTransferMachine(robot);
        outtakeMachine = StateMachines.getOuttakeMachine(robot);

        //TODO fix all controls
        //DriveControl driveControl = new DriveControl(robot, gamepad1);
        IntakeControl intakeControl = new IntakeControl(robot, gamepad1);
        //OuttakeControl outtakeControl = new OuttakeControl(robot, gamepad1);
        //VisionControl visionControl = new VisionControl(robot, gamepad1);

        StateMachine globalMachine = new StateMachineBuilder()

                .build();

        //Press Start
        waitForStart();

        robot.toInit();
        runtime.reset();
        globalMachine.start();

        //Main Loop
        while (opModeIsActive()) {
            robot.update();
            globalMachine.update();
            telemetry.update();
        }
    }
}
