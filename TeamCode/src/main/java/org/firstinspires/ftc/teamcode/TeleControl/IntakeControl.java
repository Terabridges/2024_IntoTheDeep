package org.firstinspires.ftc.teamcode.TeleControl;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utility.EdgeDetector;

public class IntakeControl implements Control{

    //Software
    IntakeSystem intake;
    Gamepad gp1;
    boolean manualIntake = true;
    Robot robot;
    EdgeDetector runIntakeRE_SSM = new EdgeDetector(() -> intakeFSM());
    EdgeDetector manualSlidesInRE = new EdgeDetector(() -> intake.intakeSlidesSetManualIn());
    EdgeDetector manualSlidesInFE = new EdgeDetector(() -> intake.intakeSlidesSetManualStop(), true);
    EdgeDetector manualSlidesOutRE = new EdgeDetector(() -> intake.intakeSlidesSetManualOut());
    EdgeDetector manualSlidesOutFE = new EdgeDetector(() -> intake.intakeSlidesSetManualStop(), true);

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
    public void intakeFSM(){

    }


    //Interface Methods
    @Override
    public void update(){
        //Set Spin with Triggers
        if (manualIntake) {
            if (gp1.right_trigger > 0.1) {
                intake.intakeSpinTarget = gp1.right_trigger;
            }

            if (gp1.left_trigger > 0.1) {
                intake.intakeSpinTarget = -gp1.right_trigger;
            }
        }

        //Intake FSM with A
        runIntakeRE_SSM.update(gp1.a);

        //dPadLeft pulls intake slides in
        manualSlidesInRE.update(gp1.dpad_left);
        manualSlidesInFE.update(gp1.dpad_left);

        //dPadRight pushes intake slides out
        manualSlidesOutRE.update(gp1.dpad_right);
        manualSlidesOutFE.update(gp1.dpad_right);
    }

    @Override
    public void addTelemetry(Telemetry telemetry){

    }


}
