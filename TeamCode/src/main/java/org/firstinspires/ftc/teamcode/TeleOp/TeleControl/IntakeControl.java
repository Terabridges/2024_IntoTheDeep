package org.firstinspires.ftc.teamcode.TeleOp.TeleControl;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utility.EdgeDetector;

public class IntakeControl implements Control{

    //Software
    IntakeSystem intake;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    EdgeDetector manualSlidesInRE = new EdgeDetector(() -> intake.intakeSlidesSetManualIn());
    EdgeDetector manualSlidesInFE = new EdgeDetector(() -> intake.intakeSlidesSetManualStop(), true);
    EdgeDetector manualSlidesOutRE = new EdgeDetector(() -> intake.intakeSlidesSetManualOut());
    EdgeDetector manualSlidesOutFE = new EdgeDetector(() -> intake.intakeSlidesSetManualStop(), true);
    EdgeDetector intakeSweeperRE = new EdgeDetector(() -> intake.intakeSweeperOut());
    EdgeDetector intakeSweeperFE = new EdgeDetector(() -> intake.intakeSweeperIn(), true);
    EdgeDetector increaseIntakeRE = new EdgeDetector(() -> intake.setIntakeHigher());
    EdgeDetector decreaseIntakeRE = new EdgeDetector(() -> intake.setIntakeLower());


    //Constructor
    public IntakeControl(IntakeSystem intake, Gamepad gp1, Gamepad gp2) {
        this.intake = intake;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public IntakeControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.intakeSystem, gp1, gp2);
        this.robot = robot;
    }

    //Methods

    //Interface Methods
    @Override
    public void update(){

        //Set Spin with Triggers
        if (intake.manualIntake) {
            if (gp1.right_trigger > 0) {
                intake.intakeSpinTarget = gp1.right_trigger;
            } else if (gp1.left_trigger > 0) {
                intake.intakeSpinTarget = -gp1.left_trigger;
            } else {
                intake.intakeSpinTarget = 0;
            }
        }

        //dPadLeft pulls intake slides in
        manualSlidesInRE.update(gp1.dpad_left);
        manualSlidesInFE.update(gp1.dpad_left);

        //dPadRight pushes intake slides out
        manualSlidesOutRE.update(gp1.dpad_right);
        manualSlidesOutFE.update(gp1.dpad_right);

        //dPadDown holds intake sweeper out when held
        intakeSweeperRE.update(gp1.dpad_down);
        intakeSweeperFE.update(gp1.dpad_down);

        increaseIntakeRE.update(gp2.dpad_right);
        decreaseIntakeRE.update(gp2.dpad_left);
    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Intake Slides Offset", intake.intakeCounter);
    }


}
