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
    ElapsedTime hangTimer = new ElapsedTime();

    //private LoopTimer loopTimer = new LoopTimer();

    public boolean isHanging = false;

    //Global FSM
    enum globalStates {
        INIT,
        INTAKE,
        TRANSFER,
        OUTTAKE,
        SPECIMEN
    }

    //Other FSMs
    public enum intakeStates {
        START,
        SWIVEL_UP,
        EXTEND,
        COLOR_WAIT,
        RETRACT,
        SWIVEL_DOWN
    }

    public enum transferStates {
        START,
        TRANSFER,
        CLOSE
    }

    public enum outtakeStates {
        START,
        OUTTAKE_RISE,
        WAIT_STATE,
        SCORE_SAMPLE,
        CLAW_OPEN
    }

    public enum specimenStates {
        START,
        SLIDES_RISE,
        SLIDES_FALL,
        PICKUP,
        RESET
    }

    public StateMachine intakeMachine;
    public StateMachine transferMachine;
    public StateMachine outtakeMachine;
    public StateMachine specimenMachine;
    public DriveControl driveControl;
    public IntakeControl intakeControl;
    public OuttakeControl outtakeControl;
    public VisionControl visionControl;
    public List<Control> controls;

    public Gamepad currentGamepad1;
    public Gamepad previousGamepad1;

    public Gamepad currentGamepad2;
    public Gamepad previousGamepad2;

    //Run OpMode
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        intakeMachine = getIntakeMachine(robot);
        transferMachine = getTransferMachine(robot);
        outtakeMachine = getOuttakeMachine(robot);
        specimenMachine = getSpecimenMachine(robot);

        driveControl = new DriveControl(robot, gamepad1, gamepad2);
        intakeControl = new IntakeControl(robot, gamepad1, gamepad2);
        outtakeControl = new OuttakeControl(robot, gamepad1, gamepad2);
        visionControl = new VisionControl(robot, gamepad1, gamepad2);

        controls = new ArrayList<>(Arrays.asList(intakeControl, outtakeControl, driveControl, visionControl));

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        StateMachine globalMachine = new StateMachineBuilder()

                // INIT/IDLE STATE
                .state(globalStates.INIT)
                .onEnter(() -> robot.currentState = "init")
                .transition( () -> aPressed(), globalStates.INTAKE)
                .transition( () -> bPressed(), globalStates.TRANSFER)
                .transition( () -> yPressed(), globalStates.OUTTAKE)
                .transition( () -> leftBumpPressed(), globalStates.SPECIMEN)

                //INTAKE FSM
                .state(globalStates.INTAKE)
                .onEnter( () -> robot.currentState = "intake")
                .transition( () -> (bPressed() && (intakeMachine.getState() == intakeStates.START)), globalStates.TRANSFER)
                .transition( () -> (yPressed() && (intakeMachine.getState() == intakeStates.START)), globalStates.OUTTAKE)
                .transition( () -> (leftBumpPressed() && (intakeMachine.getState() == intakeStates.START)), globalStates.SPECIMEN)
                .transition( () -> backPressed(), globalStates.INIT)

                //TRANSFER FSM
                .state(globalStates.TRANSFER)
                .onEnter( () -> robot.currentState = "transfer")
                .transition( () -> (aPressed() && (transferMachine.getState() == transferStates.START)), globalStates.INTAKE)
                .transition( () -> (yPressed() && (transferMachine.getState() == transferStates.START)), globalStates.OUTTAKE)
                .transition( () -> (leftBumpPressed() && (transferMachine.getState() == transferStates.START)), globalStates.SPECIMEN)
                .transition( () -> backPressed(), globalStates.INIT)

                //OUTTAKE FSM
                .state(globalStates.OUTTAKE)
                .onEnter( () -> robot.currentState = "outtake")
                .transition( () -> (aPressed() && (outtakeMachine.getState() == outtakeStates.START)), globalStates.INTAKE)
                .transition( () -> (bPressed() && (outtakeMachine.getState() == outtakeStates.START)), globalStates.TRANSFER)
                .transition( () -> (leftBumpPressed() && (outtakeMachine.getState() == outtakeStates.START)), globalStates.SPECIMEN)
                .transition( () -> backPressed(), globalStates.INIT)

                //SPECIMEN FSM
                .state(globalStates.SPECIMEN)
                .onEnter( () -> robot.currentState = "specimen")
                .transition( () -> (aPressed() && (specimenMachine.getState() == specimenStates.START)), globalStates.INTAKE)
                .transition( () -> (bPressed() && (specimenMachine.getState() == specimenStates.START)), globalStates.TRANSFER)
                .transition( () -> (yPressed() && (specimenMachine.getState() == specimenStates.START)), globalStates.OUTTAKE)
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
        specimenMachine.start();

        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //Main Loop
        while (opModeIsActive()) {
            //loopTimer.updateTime(telemetry);

            gamepadUpdate();
            robot.update();
            globalMachine.update();
            controlsUpdate();

            intakeMachine.update();
            outtakeMachine.update();
            transferMachine.update();
            specimenMachine.update();

            if (currentGamepad2.left_stick_button && !previousGamepad2.left_stick_button){
                hangMacro();
            }

            if (isHanging && (hangTimer.seconds() > 3)){
                robot.slowFall();
                robot.outtakeSystem.manualOuttake = true;
                robot.intakeSystem.manualIntake = false;
                telemetry.addData("Hang Macro", "Brake");
            }

            if (isHanging && (hangTimer.seconds() > 6)){
                telemetry.addData("Hang Macro", "Stop");
                stop();
            }

            telemetry.addData("CURRENT STATE", robot.currentState);
            telemetry.addData("OuttakeSlidesPos", robot.outtakeSystem.outtakeMiddleVertical.getCurrentPosition());
            telemetry.update();
        }
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }

    //Other Methods

    public void hangMacro(){
        isHanging = true;
        hangTimer.reset();
        telemetry.addData("Hang Macro", "start");
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

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }

    public boolean aPressed() {
        return (currentGamepad1.a && !previousGamepad1.a);
    }

    public boolean bPressed() {
        return (currentGamepad1.b && !previousGamepad1.b);
    }

    public boolean yPressed() {
        return (currentGamepad1.y && !previousGamepad1.y);
    }

    public boolean backPressed() {
        return (currentGamepad1.back && !previousGamepad1.back);
    }

    public boolean leftBumpPressed() {
        return (currentGamepad1.left_bumper && !previousGamepad1.left_bumper);
    }

    //State Machines
    public StateMachine getIntakeMachine(Robot robot){
        IntakeSystem intake = robot.intakeSystem;
        VisionSystem vision = robot.visionSystem;
        DriveSystem drive = robot.driveSystem;
        return new StateMachineBuilder()
                .state(intakeStates.START)
                .transition(() -> (!currentGamepad1.a && previousGamepad1.a) && robot.currentState.equals("intake"), intakeStates.EXTEND)

                //TODO see if this is unnecessary
//                .state(intakeStates.SWIVEL_UP)
//                .onEnter(() -> intake.intakeSwivelRest())
//                .transition(() -> intake.isSwivelRest(), intakeStates.EXTEND)

                .state(intakeStates.EXTEND)
                .onEnter( () -> {
                    intake.intakeSwivelRest();
                    intake.intakeSlidesSam();
                    drive.isIntakeExtended = true;
                })
                .transition( () -> aPressed(), intakeStates.COLOR_WAIT, () -> intake.intakeSwivelDown())

                .state(intakeStates.COLOR_WAIT)
                //TODO if past a certain point, make intake swivel rest at the same time as intake retracts. Maybe intake quarter
                .transition( () -> vision.isSomething(), intakeStates.SWIVEL_DOWN, () -> {
                    intake.intakeSwivelRest();
                    intake.intakeSlidesRetract();
                })
                .transition( () -> aPressed(), intakeStates.SWIVEL_DOWN, () -> {
                    intake.intakeSwivelRest();
                    intake.intakeSlidesRetract();
                })

//                .state(intakeStates.RETRACT)
//                .transition( () -> intake.isSwivelRest(), intakeStates.SWIVEL_DOWN, () -> intake.intakeSlidesRetract())

                .state(intakeStates.SWIVEL_DOWN)
                .onEnter(() -> {
                    //intake.intakeSlidesRetract();
                    drive.isIntakeExtended = false;
                })
                .transition(() -> intake.isIntakeRetracted(), intakeStates.START, ()-> intake.intakeSwivelTransfer())

                .build();
    }

    public StateMachine getTransferMachine(Robot robot){
        IntakeSystem intake = robot.intakeSystem;
        OuttakeSystem outtake = robot.outtakeSystem;
        return new StateMachineBuilder()
                .state(transferStates.START)
                .transition(() -> (!currentGamepad1.b && previousGamepad1.b) && robot.currentState.equals("transfer"), transferStates.TRANSFER)

                .state(transferStates.TRANSFER)
                .onEnter( () -> {
                    outtake.openClaw();
                    outtake.outtakeSwivelTransfer();
                    outtake.wristTransfer();
                    outtake.outtakeSlidesDown();
                })
                .transition(() -> outtake.isSlidesDown(), transferStates.CLOSE, () -> {
                    outtake.closeClaw();
                })

                .state(transferStates.CLOSE)
                //TODO see if transition timed is too long or short, just changed from 0.2
                .transitionTimed(0.1, transferStates.START)
                .onExit(() -> outtake.outtakeSlidesRest())

                .build();
    }

    public StateMachine getOuttakeMachine(Robot robot){
        OuttakeSystem outtake = robot.outtakeSystem;
        DriveSystem driveSystem = robot.driveSystem;
        return new StateMachineBuilder()
                .state(outtakeStates.START)
                .transition(() -> (!currentGamepad1.y && previousGamepad1.y) && robot.currentState.equals("outtake"), outtakeStates.OUTTAKE_RISE)

                .state(outtakeStates.OUTTAKE_RISE)
                .onEnter( () -> {
                    outtake.outtakeSwivelUp();
                    outtake.wristUp();
                    if (outtake.highBasketMode) {
                        outtake.outtakeSlidesHigh();
                    } else {
                        outtake.outtakeSlidesLow();
                    }
                })
                .transitionTimed(0.2, outtakeStates.WAIT_STATE)

                .state(outtakeStates.WAIT_STATE)
                .transition( () -> yPressed(), outtakeStates.SCORE_SAMPLE)

                .state(outtakeStates.SCORE_SAMPLE)
                .onEnter( () -> {
                    outtake.openClaw();
                })
                .transitionTimed(0.1, outtakeStates.CLAW_OPEN)

                .state(outtakeStates.CLAW_OPEN)
                .onEnter(() -> {
                    outtake.outtakeSwivelTransfer();
                    outtake.wristTransfer();
                    outtake.outtakeSlidesRest();
                    driveSystem.useSlowMode = false;
                    if (!(Math.abs(gamepad1.left_stick_x) > 0.05 || Math.abs(gamepad1.left_stick_y) > 0.05 || Math.abs(gamepad1.right_stick_x) > 0.05 || Math.abs(gamepad1.right_stick_y) > 0.05)) {
                        driveSystem.driveBack();
                    }
                })
                .transitionTimed(0.2, outtakeStates.START)
                .onExit(() -> driveSystem.driveStop())

                .build();
    }

    public StateMachine getSpecimenMachine(Robot robot){
        OuttakeSystem outtake = robot.outtakeSystem;
        IntakeSystem intake = robot.intakeSystem;
        DriveSystem drive = robot.driveSystem;
        return new StateMachineBuilder()
                .state(specimenStates.START)
                .transition(() -> (!currentGamepad1.left_bumper && previousGamepad1.left_bumper) && robot.currentState.equals("specimen"), specimenStates.SLIDES_RISE)

                .state(specimenStates.SLIDES_RISE)
                .onEnter(() -> {
                    outtake.wristGrab();
                    outtake.outtakeSwivelGrab();
                    outtake.openClaw();
                    outtake.isClawOpen = true;
                    intake.intakeSwivelRest();
                })
                .transition(() -> intake.isSwivelRest(), specimenStates.SLIDES_FALL)

                .state(specimenStates.SLIDES_FALL)
                .onEnter(() -> outtake.outtakeSlidesGrab1())
                .transition(() -> leftBumpPressed(), specimenStates.PICKUP)

                .state(specimenStates.PICKUP)
                .onEnter(() -> {
                    outtake.outtakeSlidesScore1();
                    outtake.wristLock();
                    outtake.outtakeSwivelLock();
                })
                .transition(() -> leftBumpPressed(), specimenStates.RESET, ()-> outtake.outtakeSlidesScore2())

                .state(specimenStates.RESET)
                .transition(() -> outtake.isSlidesScore2(), specimenStates.START, () -> {
                    outtake.openClaw();
                    outtake.isClawOpen = true;
                    outtake.outtakeSlidesRest();
                    outtake.outtakeSwivelTransfer();
                    outtake.wristTransfer();
                    drive.useSlowMode = false;
                })

                .build();
    }

}