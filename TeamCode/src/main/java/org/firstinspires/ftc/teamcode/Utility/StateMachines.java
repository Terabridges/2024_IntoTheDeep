package org.firstinspires.ftc.teamcode.Utility;

import org.firstinspires.ftc.robotcore.external.StateTransition;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.*;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.sfdev.assembly.state.*;

public class StateMachines {
    public enum intakeStates {
        EXTEND,
        COLOR_WAIT,
        RETRACT,
        FINISHED
    }

    public enum transferStates {
        INTAKE_TRANSFER,
        OUTTAKE_TRANSFER,
        OUTTAKE_RESET,
        INTAKE_RESET,
        FINISHED
    }

    public enum outtakeStates {
        OUTTAKE_RISE,
        DROP_WAIT,
        SCORE_SAMPLE,
        OUTTAKE_RESET,
        FINISHED
    }

    public static StateMachine getIntakeMachine(Robot robot){
        IntakeSystem intake = robot.intakeSystem;
        VisionSystem vision = robot.visionSystem;

        return new StateMachineBuilder()

                .state(intakeStates.EXTEND)
                .onEnter( () -> intake.intakeSlidesExtend())
                .transition( () -> intake.isIntakeExtended(), intakeStates.COLOR_WAIT, () -> intake.intakeSwivelDown())

                .state(intakeStates.COLOR_WAIT)
                .transition( () -> vision.isSomething(), intakeStates.RETRACT, () -> {
                    intake.intakeSwivelRest();
                    robot.rumble(500);

                })
                .transition( () -> robot.checkIntakeInput(), intakeStates.RETRACT, () -> intake.intakeSwivelRest())

                .state(intakeStates.RETRACT)
                .onEnter( () -> robot.intakeInput = false )
                .transition( () -> intake.isSwivelRetracted(), intakeStates.FINISHED, () -> intake.intakeSlidesRetract())

                .state(intakeStates.FINISHED)
                .build();
    }

    public static StateMachine getTransferMachine(Robot robot){
        IntakeSystem intake = robot.intakeSystem;
        OuttakeSystem outtake = robot.outtakeSystem;
        VisionSystem vision = robot.visionSystem;
        return new StateMachineBuilder()

                .state(transferStates.INTAKE_TRANSFER)
                .onEnter( () -> {
                    intake.intakeSwivelTransfer();
                    intake.intakeSlidesRetract();
                })
                .transition( () -> intake.isSwivelTransfer(), transferStates.OUTTAKE_TRANSFER, () -> {
                    outtake.openClaw();
                    outtake.outtakeSlidesDown();
                })

                .state(transferStates.OUTTAKE_TRANSFER)
                .transition( () -> outtake.isSlidesDown(), transferStates.OUTTAKE_RESET, () -> outtake.closeClaw())

                .state(transferStates.OUTTAKE_RESET)
                .onEnter( () -> outtake.outtakeSlidesRest())
                .transition( () -> !vision.isSomething(), transferStates.INTAKE_RESET, () -> robot.rumble(500))
                .transition( () -> robot.transferInput, transferStates.INTAKE_RESET)

                .state(transferStates.INTAKE_RESET)
                .onEnter( () -> robot.transferInput = false)
                .transition( () -> outtake.isSlidesRest(), transferStates.FINISHED, () -> intake.intakeSwivelRest())

                .state(transferStates.FINISHED)
                .build();
    }

    public static StateMachine getOuttakeMachine(Robot robot){
        OuttakeSystem outtake = robot.outtakeSystem;
        VisionSystem vision = robot.visionSystem;
        return new StateMachineBuilder()

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
                .onEnter( () -> vision.setLookForBasket())
                .transition( () -> robot.checkOuttakeInput(), outtakeStates.SCORE_SAMPLE)

                .state(outtakeStates.SCORE_SAMPLE)
                .onEnter( () -> {
                    outtake.openClaw();
                    robot.outtakeInput = false;
                })
                .transitionTimed(0.5, outtakeStates.OUTTAKE_RESET)

                .state(outtakeStates.OUTTAKE_RESET)
                .onEnter( () -> {
                    outtake.outtakeSwivelDown();
                    outtake.outtakeSlidesRest();
                })
                .transition( () -> outtake.isSlidesRest(), outtakeStates.FINISHED)

                .state(outtakeStates.FINISHED)
                .build();
    }
}