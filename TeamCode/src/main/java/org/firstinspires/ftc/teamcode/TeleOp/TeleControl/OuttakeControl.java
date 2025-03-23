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
    Gamepad gp2;
    Robot robot;
    EdgeDetector slidesManualRE = new EdgeDetector( () -> robot.setManualSlidesTrue());
    EdgeDetector basketMode = new EdgeDetector( () -> toggleLowBasketMode());
    EdgeDetector clawRE = new EdgeDetector( () -> toggleClaw());
    EdgeDetector resetEncodersRE = new EdgeDetector(() -> outtake.resetEncodersButton());
    EdgeDetector useLimitSwitchRE = new EdgeDetector(() -> robot.toggleLimitSwitch());
    EdgeDetector increaseOuttakeRE = new EdgeDetector(() -> outtake.setOuttakeHigher());
    EdgeDetector decreaseOuttakeRE = new EdgeDetector(() -> outtake.setOuttakeLower());

    //Constructor
    public OuttakeControl(OuttakeSystem outtake, Gamepad gp1, Gamepad gp2){
        this.outtake = outtake;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public OuttakeControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.outtakeSystem, gp1, gp2);
        this.robot = robot;
    }

    //Methods

    public void toggleLowBasketMode() {
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
            } else if (gp2.right_trigger > 0) {
                outtake.outtakeSetSlides(gp2.right_trigger);
            } else if (gp2.left_trigger > 0) {
                outtake.outtakeSetSlides(-gp2.left_trigger);
            } else {
                //outtake.usePIDF = true;
                outtake.outtakeSetSlides(0);
            }
        }

        //Outtake manual with right stick
        slidesManualRE.update(gp1.right_stick_button || gp2.right_stick_button);

        //GAMEPAD2 basketModeToggle A
        basketMode.update(gp2.a);

        //Claw open/close with right bumper
        clawRE.update(gp1.right_bumper);

        //GAMEPAD 2 Reset slide encoders with right bumper
        resetEncodersRE.update(gp2.right_bumper);

        //GAMEPAD 2 Use limit switch toggle X
        useLimitSwitchRE.update(gp2.x);

        increaseOuttakeRE.update(gp2.dpad_up);
        decreaseOuttakeRE.update(gp2.dpad_down);
    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Basket Mode", (outtake.highBasketMode ? "HIGH" : "LOW"));
        telemetry.addData("Manual Slides", outtake.manualOuttake);
        telemetry.addData("Slides Pos", outtake.outtakeBottomVertical.getCurrentPosition());
        //telemetry.addData("Slides Mode", outtake.outtakeBottomVertical.getMode());
        //telemetry.addData("Zero Power Behavior", outtake.outtakeBottomVertical.getZeroPowerBehavior());
        telemetry.addData("Use Limit Switch", outtake.useLimitSwitch);
        telemetry.addData("Outtake Slides Offset", outtake.outtakeCounter);
    }

}
