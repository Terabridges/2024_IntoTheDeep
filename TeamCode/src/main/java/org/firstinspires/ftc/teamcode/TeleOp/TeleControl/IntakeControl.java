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
    Robot robot;
    EdgeDetector manualSlidesInRE = new EdgeDetector(() -> intake.intakeSlidesSetManualIn());
    EdgeDetector manualSlidesInFE = new EdgeDetector(() -> intake.intakeSlidesSetManualStop(), true);
    EdgeDetector manualSlidesOutRE = new EdgeDetector(() -> intake.intakeSlidesSetManualOut());
    EdgeDetector manualSlidesOutFE = new EdgeDetector(() -> intake.intakeSlidesSetManualStop(), true);
    EdgeDetector intakeSweeperRE = new EdgeDetector(() -> intake.intakeSweeperOut());
    EdgeDetector intakeSweeperFE = new EdgeDetector(() -> intake.intakeSweeperIn(), true);


    //Constructor
    public IntakeControl(IntakeSystem intake, Gamepad gp1) {
        this.intake = intake;
        this.gp1 = gp1;
    }

    public IntakeControl(Robot robot, Gamepad gp1) {
        this(robot.intakeSystem, gp1);
        this.robot = robot;
    }

    //Methods

    //Interface Methods
    @Override
    public void update(){

        //Set Spin with Triggers
        if (intake.manualIntake) {
            if (gp1.right_trigger > 0.1) {
                intake.intakeSpinTarget = gp1.right_trigger;
            }

            if (gp1.left_trigger > 0.1) {
                intake.intakeSpinTarget = -gp1.left_trigger;
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
    }

    @Override
    public void addTelemetry(Telemetry telemetry){

    }


}
