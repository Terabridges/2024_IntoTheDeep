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
    EdgeDetector slidesManualRE = new EdgeDetector( () -> robot.setManualSlidesTrue());
    EdgeDetector basketMode = new EdgeDetector( () -> toggleLowBasketMode());
    EdgeDetector clawRE = new EdgeDetector( () -> toggleClaw());
    EdgeDetector resetEncodersRE = new EdgeDetector(() -> outtake.resetSlideEncoders());

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

    public void toggleClaw(){
        outtake.isClawOpen = !outtake.isClawOpen;
        if (outtake.isClawOpen){
            outtake.closeClaw();
        } else {
            outtake.openClaw();
        }
    }

    //Interface Methods
    @Override
    public void update(){

        //Set slides power with triggers
        if (outtake.manualOuttake) {
            if (gp1.right_trigger > 0) {
                outtake.outtakeSetSlides(gp1.right_trigger);
            } else if (gp1.left_trigger > 0) {
                outtake.outtakeSetSlides(-gp1.left_trigger);
            } else {
                outtake.outtakeSetSlides(0);
            }
        }

        //Outtake manual with right stick
        slidesManualRE.update(gp1.right_stick_button);

        //Basket Mode Toggle
        basketMode.update(gp1.left_stick_button);

        //Claw open/close with right bumper
        clawRE.update(gp1.right_bumper);

        //Reset slide encoders with start button
        resetEncodersRE.update(gp1.start);
    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Basket Mode", (outtake.highBasketMode ? "HIGH" : "LOW"));
        telemetry.addData("Manual Slides", outtake.manualOuttake);
        telemetry.addData("Slides Pos", outtake.outtakeBottomVertical.getCurrentPosition());
    }

}
