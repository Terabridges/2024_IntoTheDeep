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
    EdgeDetector intakeInput = new EdgeDetector(() -> setIntakeInput());
    EdgeDetector transferInput = new EdgeDetector(() -> setTransferInput());
    EdgeDetector outtakeInput = new EdgeDetector(() -> setOuttakeInput());


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
    public void setIntakeInput(){
        if (robot.currentState.equals("intake")) {robot.intakeInput = true;}
    }

    public void setTransferInput(){
        if (robot.currentState.equals("transfer")) {robot.transferInput = true;}
    }

    public void setOuttakeInput(){
        if (robot.currentState.equals("outtake")) {robot.outtakeInput = true;}
    }

    //Interface Methods
    @Override
    public void update(){

        //Set Spin with Triggers
        if (intake.manualIntake) {
            if (gp1.right_trigger > 0.1) {
                intake.intakeSpinTarget = gp1.right_trigger;
            }

            if (gp1.left_trigger > 0.1) {
                intake.intakeSpinTarget = -gp1.right_trigger;
            }
        }

        //dPadLeft pulls intake slides in
        manualSlidesInRE.update(gp1.dpad_left);
        manualSlidesInFE.update(gp1.dpad_left);

        //dPadRight pushes intake slides out
        manualSlidesOutRE.update(gp1.dpad_right);
        manualSlidesOutFE.update(gp1.dpad_right);

        //Inputs
        intakeInput.update(gp1.x);
        transferInput.update(gp1.x);
        outtakeInput.update(gp1.x);
    }

    @Override
    public void addTelemetry(Telemetry telemetry){

    }


}
