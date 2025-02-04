package org.firstinspires.ftc.teamcode.TeleOp.TeleControl;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utility.EdgeDetector;

public class OuttakeControl implements Control {

    //Software
    OuttakeSystem outtake;
    Gamepad gp1;
    Robot robot;
    boolean isClawOpen = false;
    EdgeDetector clawOpenRE = new EdgeDetector( () -> outtake.openClaw());
    EdgeDetector clawCloseRE = new EdgeDetector( () -> outtake.closeClaw());
    EdgeDetector slidesManualRE = new EdgeDetector( () -> robot.setManualSlidesTrue());
    EdgeDetector slidesManualFalseRE = new EdgeDetector( () -> robot.setManualIntakeTrue());
    EdgeDetector basketMode = new EdgeDetector( () -> toggleLowBasketMode());

    //Constructor
    public OuttakeControl(OuttakeSystem outtake, Gamepad gp1){
        this.outtake = outtake;
        this.gp1 = gp1;
    }

    public OuttakeControl(Robot robot, Gamepad gp1) {
        this(robot.outtakeSystem, gp1);
        this.robot = robot;
    }

    //Methods
    public void toggleLowBasketMode(){
        outtake.highBasketMode = !outtake.highBasketMode;
    }

    //Interface Methods
    @Override
    public void update(){

        //Set slides power with triggers
        if (outtake.manualOuttake) {
            if (gp1.right_trigger > 0.1) {
                outtake.outtakeSetSlides(gp1.right_trigger);
            }

            if (gp1.left_trigger > 0.1) {
                outtake.outtakeSetSlides(-gp1.right_trigger);
            }
        }

        //Claw open/close with dpad up
        if (isClawOpen){
            clawCloseRE.update(gp1.dpad_up);
        } else {
            clawOpenRE.update(gp1.dpad_up);
        }

        //Outtake manual with right stick
        if (outtake.manualOuttake){
            slidesManualFalseRE.update(gp1.right_stick_button);
        } else {
            slidesManualRE.update(gp1.right_stick_button);
        }

        //Basket Mode Toggle
        basketMode.update(gp1.left_stick_button);
    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Basket Mode", (outtake.highBasketMode ? "HIGH" : "LOW"));
    }

}
