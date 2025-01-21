package org.firstinspires.ftc.teamcode.Utility;

import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.robotcore.external.StateTransition;

import org.firstinspires.ftc.teamcode.Subsystems.*;

public class StateMachines {
    public enum intakeStates {
        START,
        EXTEND,
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
        LIGHT_WAIT,
        SCORE_SAMPLE
    }

//    public static StateMachine getIntakeMachine(Robot robot){
//
//    }

    //StateMachine machine = new StateMachineBuilder
}
