package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.TeleOp.TeleControl.*;
import org.firstinspires.ftc.teamcode.Utility.*;

import com.sfdev.assembly.state.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

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

    //Other FSMs
    public enum intakeStates {
        START,
        EXTEND,
        COLOR_WAIT,
        RETRACT
    }

    public enum transferStates {
        START,
        INTAKE_TRANSFER,
        OUTTAKE_TRANSFER,
        OUTTAKE_RESET,
        INTAKE_RESET
    }

    public enum outtakeStates {
        START,
        OUTTAKE_RISE,
        DROP_WAIT,
        SCORE_SAMPLE,
        OUTTAKE_RESET
    }

    public StateMachine intakeMachine;
    public StateMachine transferMachine;
    public StateMachine outtakeMachine;
    public DriveControl driveControl;
    public IntakeControl intakeControl;
    public OuttakeControl outtakeControl;
    //public VisionControl visionControl;
    public List<Control> controls;

    public Gamepad currentGamepad1;
    public Gamepad previousGamepad1;

    //Run OpMode
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, telemetry, gamepad1);

        intakeMachine = getIntakeMachine(robot);
        transferMachine = getTransferMachine(robot);
        outtakeMachine = getOuttakeMachine(robot);

        driveControl = new DriveControl(robot, gamepad1);
        intakeControl = new IntakeControl(robot, gamepad1);
        outtakeControl = new OuttakeControl(robot, gamepad1);
        //visionControl = new VisionControl(robot, gamepad1);

        controls = new ArrayList<>(Arrays.asList(intakeControl, outtakeControl, driveControl));

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        StateMachine globalMachine = new StateMachineBuilder()

                // INIT/IDLE STATE
                .state(globalStates.INIT)
                .onEnter(() -> robot.currentState = "init")
                .transition( () -> aPressed(), globalStates.INTAKE)
                .transition( () -> bPressed(), globalStates.TRANSFER)
                .transition( () -> yPressed(), globalStates.OUTTAKE)

                //INTAKE FSM
                .state(globalStates.INTAKE)
                .onEnter( () -> robot.currentState = "intake")
                .transition( () -> (bPressed() && (intakeMachine.getState() == intakeStates.START)), globalStates.TRANSFER)
                .transition( () -> (yPressed() && (intakeMachine.getState() == intakeStates.START)), globalStates.OUTTAKE)
                .transition( () -> backPressed(), globalStates.INIT)

                //TRANSFER FSM
                .state(globalStates.TRANSFER)
                .onEnter( () -> robot.currentState = "transfer")
                .transition( () -> (aPressed() && (transferMachine.getState() == transferStates.START)), globalStates.INTAKE)
                .transition( () -> (yPressed() && (transferMachine.getState() == transferStates.START)), globalStates.OUTTAKE)
                .transition( () -> backPressed(), globalStates.INIT)

                //OUTTAKE FSM
                .state(globalStates.OUTTAKE)
                .onEnter( () -> robot.currentState = "outtake")
                .transition( () -> (aPressed() && (outtakeMachine.getState() == outtakeStates.START)), globalStates.INTAKE)
                .transition( () -> (bPressed() && (outtakeMachine.getState() == outtakeStates.START)), globalStates.TRANSFER)
                .transition( () -> backPressed(), globalStates.INIT)

                .build();

        //Press Start
        waitForStart();

        robot.toInit();
        runtime.reset();
        globalMachine.start();
        intakeMachine.start();
        outtakeMachine.start();
        transferMachine.start();
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //Main Loop
        while (opModeIsActive()) {
            gamepadUpdate();
            robot.update();
            globalMachine.update();
            controlsUpdate();

            intakeMachine.update();
            outtakeMachine.update();
            transferMachine.update();

            telemetry.addData("CURRENT STATE", robot.currentState);
            telemetry.addData("INTAKE STATE", intakeMachine.getState());
            telemetry.addData("OUTTAKE STATE", outtakeMachine.getState());
            telemetry.addData("TRANSFER STATE", transferMachine.getState());
            telemetry.update();
        }
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    }

    public void controlsUpdate() {
        for (Control c : controls) {
            c.update();
            c.addTelemetry(telemetry);
        }
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
    }

    public boolean aPressed() {
        return (currentGamepad1.a && !previousGamepad1.a);
    }

    public boolean bPressed() {
        return (currentGamepad1.b && !previousGamepad1.b);
    }

    public boolean xPressed() {
        return (currentGamepad1.x && !previousGamepad1.x);
    }

    public boolean yPressed() {
        return (currentGamepad1.y && !previousGamepad1.y);
    }

    public boolean backPressed() {
        return (currentGamepad1.back && !previousGamepad1.back);
    }

    public StateMachine getIntakeMachine(Robot robot){
        IntakeSystem intake = robot.intakeSystem;
        //VisionSystem vision = robot.visionSystem;
        return new StateMachineBuilder()
                .state(intakeStates.START)
                .transition(() -> (!currentGamepad1.a && previousGamepad1.a) && robot.currentState.equals("intake"), intakeStates.EXTEND)

                .state(intakeStates.EXTEND)
                .onEnter( () -> intake.intakeSlidesExtend())
                .transition( () -> intake.isIntakeExtended(), intakeStates.COLOR_WAIT, () -> intake.intakeSwivelDown())

                .state(intakeStates.COLOR_WAIT)
//                .transition( () -> vision.isSomething(), intakeStates.RETRACT, () -> {
//                    intake.intakeSwivelRest();
//                    robot.rumble(500);
//
//                })
                .transition( () -> xPressed(), intakeStates.RETRACT, () -> intake.intakeSwivelTransfer())

                .state(intakeStates.RETRACT)
                .transition( () -> intake.isSwivelTransfer(), intakeStates.START, () -> intake.intakeSlidesRetract())

                .build();
    }

    public StateMachine getTransferMachine(Robot robot){
        IntakeSystem intake = robot.intakeSystem;
        OuttakeSystem outtake = robot.outtakeSystem;
        //VisionSystem vision = robot.visionSystem;
        return new StateMachineBuilder()
                .state(transferStates.START)
                .transition(() -> (!currentGamepad1.b && previousGamepad1.b) && robot.currentState.equals("transfer"), transferStates.INTAKE_TRANSFER)

                .state(transferStates.INTAKE_TRANSFER)
                .onEnter( () -> {
                    intake.intakeSwivelTransfer();
                    intake.intakeSlidesRetract();
                    outtake.openClaw();
                })
                .transition( () -> intake.isSwivelTransfer(), transferStates.OUTTAKE_TRANSFER, () -> {
                    outtake.outtakeSwivelTransfer();
                    outtake.outtakeSlidesDown();
                    outtake.wristTransfer();
                })

                .state(transferStates.OUTTAKE_TRANSFER)
                .transition( () -> outtake.isSlidesDown(), transferStates.OUTTAKE_RESET, () -> outtake.closeClaw())

                .state(transferStates.OUTTAKE_RESET)
                .transitionTimed(0.5, transferStates.INTAKE_RESET)

                .state(transferStates.INTAKE_RESET)
                .onEnter( () -> outtake.outtakeSlidesRest())
                .transition( () -> outtake.isSlidesRest(), transferStates.START, () -> {
                    intake.intakeSwivelTransfer();
                    outtake.outtakeSwivelDown();
                    outtake.wristDown();
                })

                .build();
    }

    public StateMachine getOuttakeMachine(Robot robot){
        OuttakeSystem outtake = robot.outtakeSystem;
        //VisionSystem vision = robot.visionSystem;
        return new StateMachineBuilder()
                .state(outtakeStates.START)
                .transition(() -> (!currentGamepad1.y && previousGamepad1.y) && robot.currentState.equals("outtake"), outtakeStates.OUTTAKE_RISE)

                .state(outtakeStates.OUTTAKE_RISE)
                .onEnter( () -> {
                    if (outtake.highBasketMode) {
                        outtake.outtakeSlidesHigh();
                    } else {
                        outtake.outtakeSlidesLow();
                    }
                    outtake.outtakeSwivelUp();
                })
                .transition( () -> ((outtake.highBasketMode && outtake.isSlidesHigh()) || (!outtake.highBasketMode && outtake.isSlidesLow())), outtakeStates.DROP_WAIT)

                .state(outtakeStates.DROP_WAIT)
                //.onEnter( () -> vision.setLookForBasket())
                .transition( () -> xPressed(), outtakeStates.SCORE_SAMPLE)

                .state(outtakeStates.SCORE_SAMPLE)
                .onEnter( () -> outtake.openClaw())
                .transitionTimed(0.5, outtakeStates.OUTTAKE_RESET)

                .state(outtakeStates.OUTTAKE_RESET)
                .onEnter( () -> {
                    outtake.outtakeSwivelDown();
                    outtake.outtakeSlidesRest();
                    outtake.closeClaw();
                })
                .transitionTimed( 0.75, outtakeStates.START)

                .build();
    }

}
