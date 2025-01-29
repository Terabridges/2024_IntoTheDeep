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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

@Disabled
@TeleOp(name="MainTeleOp", group="TeleOp")
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

    public StateMachine intakeMachine;
    public StateMachine transferMachine;
    public StateMachine outtakeMachine;
    public DriveControl driveControl;
    public IntakeControl intakeControl;
    public OuttakeControl outtakeControl;
    public VisionControl visionControl;
    public List<Control> controls;

    //Run OpMode
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, telemetry, gamepad1);
        GamepadDetector gpDetec = new GamepadDetector(this);

        intakeMachine = StateMachines.getIntakeMachine(robot);
        transferMachine = StateMachines.getTransferMachine(robot);
        outtakeMachine = StateMachines.getOuttakeMachine(robot);

        driveControl = new DriveControl(robot, gamepad1);
        intakeControl = new IntakeControl(robot, gamepad1);
        outtakeControl = new OuttakeControl(robot, gamepad1);
        //visionControl = new VisionControl(robot, gamepad1);

        controls = new ArrayList<>(Arrays.asList(intakeControl, outtakeControl, driveControl));

        StateMachine globalMachine = new StateMachineBuilder()

                // INIT/IDLE STATE
                .state(globalStates.INIT)
                .transition( () -> gpDetec.aPressed(), globalStates.INTAKE)
                .transition( () -> gpDetec.bPressed(), globalStates.TRANSFER)
                .transition( () -> gpDetec.yPressed(), globalStates.OUTTAKE)

                //INTAKE FSM
                .state(globalStates.INTAKE)
                .onEnter( () -> {
                    intakeMachine.reset();
                    intakeMachine.start();
                    robot.currentState = "intake";
                })
                .loop( () -> intakeMachine.update())
                .onExit( () -> intakeMachine.stop())
                .transition( () -> gpDetec.bPressed(), globalStates.TRANSFER)
                .transition( () -> gpDetec.yPressed(), globalStates.OUTTAKE)

                //TRANSFER FSM
                .state(globalStates.TRANSFER)
                .onEnter( () -> {
                    transferMachine.reset();
                    transferMachine.start();
                    robot.currentState = "transfer";
                })
                .loop( () -> transferMachine.update())
                .onExit( () -> transferMachine.stop())
                .transition( () -> gpDetec.aPressed(), globalStates.INTAKE)
                .transition( () -> gpDetec.yPressed(), globalStates.OUTTAKE)

                //OUTTAKE FSM
                .state(globalStates.OUTTAKE)
                .onEnter( () -> {
                    outtakeMachine.reset();
                    outtakeMachine.start();
                    robot.currentState = "outtake";
                })
                .loop( () -> outtakeMachine.update())
                .onExit( () -> outtakeMachine.stop())
                .transition( () -> gpDetec.aPressed(), globalStates.INTAKE)
                .transition( () -> gpDetec.bPressed(), globalStates.TRANSFER)

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
            gpDetec.update();
            ControlsUpdate();
            telemetry.update();
        }

    }

    public void ControlsUpdate() {
        for (Control c : controls) {
            c.update();
        }
    }

    public void pleasePark(Runtime runtime) {

    }
}
