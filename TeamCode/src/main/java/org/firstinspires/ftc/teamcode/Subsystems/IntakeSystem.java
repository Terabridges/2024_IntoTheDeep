package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utility.AbsoluteAnalogEncoder;

@Config
public class IntakeSystem implements Subsystem {

    //Hardware
    public CRServo intakeLeftSlide;
    public CRServo intakeRightSlide;
    public CRServo intakeRightSwivel;
    public CRServo intakeLeftSwivel;
    public DcMotor intakeSpin;
    public AnalogInput intakeRightSwivelAnalog;
    public AnalogInput intakeRightSlidesAnalog;
    public AbsoluteAnalogEncoder intakeRightSwivelEnc;
    public AbsoluteAnalogEncoder intakeRightSlidesEnc;

    //SOFTWARE
    public boolean usePIDFIntakeSlides = true;
    public boolean usePIDFIntakeSwivel = true;
    public boolean manualIntake = true;
    private int servoOffset = 0;
    private int motorOffset = 0;

    //Positions
    private double INTAKE_SPIN_IN = -0.4;
    private double INTAKE_SPIN_OUT = 0.4;
    private double INTAKE_SPIN_STOP = 0;
    private int INTAKE_SLIDES_EXTEND = 227;
    private int INTAKE_SLIDES_RETRACT = 190;
    private int INTAKE_SWIVEL_TRANSFER;
    private int INTAKE_SWIVEL_DOWN;
    private int INTAKE_SWIVEL_REST;
    private double INTAKE_SLIDES_MANUAL_OUT = 0.3;
    private double INTAKE_SLIDES_MANUAL_IN = -0.3;
    private double INTAKE_SLIDES_MANUAL_STOP = 0;

    //Targets
    public int intakeSlidesTarget;
    public int intakeSwivelTarget;
    public double intakeSpinTarget = 0;
    public double intakeSlidesManualPower;
    public double intakeSwivelManualPower;

    //Max
    private double INTAKE_SLIDES_MAX_POWER = 1.0;
    private double INTAKE_SWIVEL_MAX_POWER = 1.0;
    private double INTAKE_SPIN_MAX_POWER = 1.0;

    //PIDF



    //Constructor
    public IntakeSystem(HardwareMap map) {
        intakeLeftSlide = map.get(CRServo.class, "intake_left_linear");
        intakeRightSlide = map.get(CRServo.class, "intake_right_linear");
        intakeLeftSwivel = map.get(CRServo.class, "intake_left_swivel");
        intakeRightSwivel = map.get(CRServo.class, "intake_right_swivel");
        intakeSpin = map.get(DcMotor.class, "intake_spin");
        intakeRightSwivelAnalog = map.get(AnalogInput.class, "intake_right_swivel_analog");
        intakeRightSlidesAnalog = map.get(AnalogInput.class, "intake_right_linear_analog");
        intakeRightSlidesEnc = new AbsoluteAnalogEncoder(intakeRightSlidesAnalog, 3.3, 0);
        intakeRightSwivelEnc = new AbsoluteAnalogEncoder(intakeRightSwivelAnalog, 3.3, 0);

    }

    //METHODS
    //Core Methods
    public void intakeSetSlides(double pow) {
        if(pow > INTAKE_SLIDES_MAX_POWER) pow = INTAKE_SLIDES_MAX_POWER;
        if(pow < -INTAKE_SLIDES_MAX_POWER) pow = -INTAKE_SLIDES_MAX_POWER;
        intakeLeftSlide.setPower(pow);
        intakeRightSlide.setPower(pow);
    }

    public void intakeSetSwivel(double pow) {
        if(pow > INTAKE_SWIVEL_MAX_POWER) pow = INTAKE_SWIVEL_MAX_POWER;
        if(pow < -INTAKE_SWIVEL_MAX_POWER) pow = -INTAKE_SWIVEL_MAX_POWER;
        intakeLeftSwivel.setPower(pow);
        intakeRightSwivel.setPower(pow);
    }

    public void intakeSetSpin(double pow) {
        if(pow > INTAKE_SPIN_MAX_POWER) pow = INTAKE_SPIN_MAX_POWER;
        if(pow < -INTAKE_SPIN_MAX_POWER) pow = -INTAKE_SPIN_MAX_POWER;
        intakeSpin.setPower(pow);
    }

    //Other Methods
    public void intakeSlidesExtend() {
        intakeSlidesTarget = INTAKE_SLIDES_EXTEND;
    }

    public void intakeSlidesRetract() {
        intakeSlidesTarget = INTAKE_SLIDES_RETRACT;
    }

    public void intakeSwivelRest() {
        intakeSwivelTarget = INTAKE_SWIVEL_REST;
    }

    public void intakeSwivelDown(){
        intakeSwivelTarget = INTAKE_SWIVEL_DOWN;
    }

    public void intakeSwivelTransfer(){
        intakeSwivelTarget = INTAKE_SWIVEL_TRANSFER;
    }

    public void intakeSpinIn() {
        intakeSpinTarget = INTAKE_SPIN_IN;
    }

    public void intakeSpinOut() {
        intakeSpinTarget = INTAKE_SPIN_OUT;
    }

    public void intakeStopSpin() {
        intakeSpinTarget = INTAKE_SPIN_STOP;
    }

    public void intakeSlidesSetManualIn(){intakeSlidesManualPower = INTAKE_SLIDES_MANUAL_IN;}

    public void intakeSlidesSetManualOut(){intakeSlidesManualPower = INTAKE_SLIDES_MANUAL_OUT;}

    public void intakeSlidesSetManualStop(){intakeSlidesManualPower = INTAKE_SLIDES_MANUAL_STOP;}

    //isPositions

    public boolean isIntakeExtended(){
        return Math.abs(intakeRightSlidesEnc.getCurrentPosition() - INTAKE_SLIDES_EXTEND) <= servoOffset;
    }

    public boolean isSwivelRetracted(){
        return Math.abs(intakeRightSwivelEnc.getCurrentPosition() - INTAKE_SWIVEL_REST) <= servoOffset;
    }

    public boolean isSwivelTransfer(){
        return Math.abs(intakeRightSwivelEnc.getCurrentPosition() - INTAKE_SWIVEL_TRANSFER) <= servoOffset;
    }

    //PIDF
    private int setIntakeSlidesPIDF(int target) {
        return 0;
    }

    private int setIntakeSwivelPIDF(int target) {
        return 0;
    }

    //Interface Methods
    @Override
    public void toInit(){
        intakeSlidesRetract();
        intakeSwivelRest();
        intakeStopSpin();
    }

    @Override
    public void update(){
        if (usePIDFIntakeSlides) {
            intakeSetSlides(setIntakeSlidesPIDF(intakeSlidesTarget));
        } else {
            intakeSetSlides(intakeSlidesManualPower);
        }
        if (usePIDFIntakeSwivel) {
            intakeSetSwivel(setIntakeSwivelPIDF(intakeSwivelTarget));
        } else {
            intakeSetSwivel(intakeSwivelManualPower);
        }
        intakeSetSpin(intakeSpinTarget);
    }

}
