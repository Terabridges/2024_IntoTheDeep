package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utility.AbsoluteAnalogEncoder;

@Config
public class IntakeSystem implements Subsystem {

    //Hardware
    public CRServo intakeLeftSlide;
    public CRServo intakeRightSlide;
    public CRServo intakeRightSwivel;
    public CRServo intakeLeftSwivel;
    public DcMotor intakeSpin;
    public Servo intakeSweeper;
    public AnalogInput intakeSwivelAnalog;
    public AnalogInput intakeSlidesAnalog;
    public AbsoluteAnalogEncoder intakeSwivelEnc;
    public AbsoluteAnalogEncoder intakeSlidesEnc;

    //SOFTWARE
    public boolean usePIDFIntakeSlides = true;
    public boolean usePIDFIntakeSwivel = true;
    public boolean manualIntake = true;
    private int servoOffset = 10;
    private int lowServoOffset = 4;
    private int motorOffset = 40;
    public double intakeSwivelGearRatio = 40.0/48.0;
    private double intakeSwivelOffset = 102.0;

    //Positions
    private double INTAKE_SPIN_IN = -0.90;
    private double INTAKE_SPIN_OUT = 0.90;
    private double INTAKE_SPIN_STOP = 0;
    private int INTAKE_SLIDES_EXTEND = 335;
    private int INTAKE_SLIDES_HALF = 325;
    private int INTAKE_SLIDES_RETRACT = 275;
    private int INTAKE_SWIVEL_TRANSFER = 124;
    private int INTAKE_SWIVEL_REST = 210;
    private int INTAKE_SWIVEL_DOWN = 293;
    private double INTAKE_SLIDES_MANUAL_OUT = 0.3;
    private double INTAKE_SLIDES_MANUAL_IN = -0.3;
    private double INTAKE_SLIDES_MANUAL_STOP = 0;
    private double INTAKE_SWEEPER_OUT = 0.65;
    private double INTAKE_SWEEPER_IN = 0.26;

    //Targets
    public double intakeSpinTarget = 0;
    public double intakeSlidesManualPower;
    public double intakeSwivelManualPower;

    //Max
    private double INTAKE_SLIDES_MAX_POWER = 1.0;
    private double INTAKE_SWIVEL_MAX_POWER = 1.0;
    private int SLIDES_MAX = 335;

    //PIDF

    private double ticks_in_degree = 144.0 / 180.0;

    //First PID for intake slides
    public PIDController intakeSlidesController;
    public static double p = 0.0075, i = 0.04, d = 0.0003;
    public static double f = 0.0;
    public static int intakeSlidesTarget;
    double intakeSlidesPos;
    double pid, targetIntakeSlidesAngle, ff, currentIntakeSlidesAngle, intakeSlidesPower;

    //Second PID for intake swivel
    public PIDController intakeSwivelController;
    public static double p2 = 0.005, i2 = 0.03, d2 = 0.00005;
    public static double f2 = 0.05;
    public static int intakeSwivelTarget;
    double intakeSwivelPos;
    double pid2, targetIntakeSwivelAngle, ff2, currentIntakeSwivelAngle, intakeSwivelPower;


    //Constructor
    public IntakeSystem(HardwareMap map) {
        intakeLeftSlide = map.get(CRServo.class, "intake_left_slide");
        intakeRightSlide = map.get(CRServo.class, "intake_right_slide");
        intakeLeftSwivel = map.get(CRServo.class, "intake_left_swivel");
        intakeRightSwivel = map.get(CRServo.class, "intake_right_swivel");
        intakeSpin = map.get(DcMotor.class, "intake_spin");
        intakeSwivelAnalog = map.get(AnalogInput.class, "intake_right_swivel_analog");
        intakeSlidesAnalog = map.get(AnalogInput.class, "intake_right_slide_analog");
        intakeSweeper = map.get(Servo.class, "intake_sweeper");
        intakeSlidesEnc = new AbsoluteAnalogEncoder(intakeSlidesAnalog);
        intakeSwivelEnc = new AbsoluteAnalogEncoder(intakeSwivelAnalog, 3.3, intakeSwivelOffset, intakeSwivelGearRatio);

        intakeLeftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRightSwivel.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeSlidesController = new PIDController(p, i, d);
        intakeSwivelController = new PIDController(p2, i2, d2);
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
        intakeSpin.setPower(pow);
    }

    public void setSweeper(double pos) {intakeSweeper.setPosition(pos);}

    //Other Methods
    public void intakeSlidesExtend() {
        intakeSlidesTarget = INTAKE_SLIDES_EXTEND;
    }

    public void intakeSlidesRetract() {
        intakeSlidesTarget = INTAKE_SLIDES_RETRACT;
    }

    public void intakeSlidesHalf() {
        intakeSlidesTarget = INTAKE_SLIDES_HALF;
    }

    public void intakeSwivelDown(){intakeSwivelTarget = INTAKE_SWIVEL_DOWN;}

    public void intakeSwivelRest(){
        intakeSwivelTarget = INTAKE_SWIVEL_REST;
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

    public void intakeSlidesSetManualIn(){
        usePIDFIntakeSlides = false;
        intakeSlidesManualPower = INTAKE_SLIDES_MANUAL_IN;
    }

    public void intakeSlidesSetManualOut(){
        usePIDFIntakeSlides = false;
        intakeSlidesManualPower = INTAKE_SLIDES_MANUAL_OUT;
    }

    public void intakeSlidesSetManualStop(){
        usePIDFIntakeSlides = true;
        intakeSlidesTarget = (int)intakeSlidesEnc.getCurrentPosition();
        intakeSlidesManualPower = INTAKE_SLIDES_MANUAL_STOP;
    }

    public void intakeSweeperOut(){setSweeper(INTAKE_SWEEPER_OUT);}

    public void intakeSweeperIn(){setSweeper(INTAKE_SWEEPER_IN);}

    //isPositions

    public boolean isIntakeExtended(){
        return Math.abs(intakeSlidesEnc.getCurrentPosition() - INTAKE_SLIDES_EXTEND) <= lowServoOffset;
    }

    public boolean isIntakeRetracted(){
        return Math.abs(intakeSlidesEnc.getCurrentPosition() - INTAKE_SLIDES_RETRACT) <= lowServoOffset;
    }

    public boolean isIntakeHalf(){
        return Math.abs(intakeSlidesEnc.getCurrentPosition() - INTAKE_SLIDES_HALF) <= lowServoOffset;
    }

    public boolean isSwivelTransfer(){
        return Math.abs(intakeSwivelEnc.getCurrentPosition() - INTAKE_SWIVEL_TRANSFER) <= servoOffset;
    }

    public boolean isSwivelRest(){
        return Math.abs(intakeSwivelEnc.getCurrentPosition() - INTAKE_SWIVEL_REST) <= servoOffset;
    }

    public double setIntakeSlidesPIDF(int target) {
        intakeSlidesController.setPID(p, i, d);
        intakeSlidesPos = intakeSlidesEnc.getCurrentPosition();
        pid = intakeSlidesController.calculate(intakeSlidesPos, target);
        targetIntakeSlidesAngle = target;
        ff = (Math.sin(Math.toRadians(targetIntakeSlidesAngle))) * f;
        currentIntakeSlidesAngle = Math.toRadians((intakeSlidesPos) / ticks_in_degree);

        intakeSlidesPower = pid + ff;

        return intakeSlidesPower;
    }

    public double setIntakeSwivelPIDF(int target) {
        intakeSwivelController.setPID(p2, i2, d2);
        intakeSwivelPos = intakeSwivelEnc.getCurrentPosition();
        pid2 = intakeSwivelController.calculate(intakeSwivelPos, target);
        targetIntakeSwivelAngle = target;
        ff2 = (Math.sin(Math.toRadians(targetIntakeSwivelAngle))) * f2;
        currentIntakeSwivelAngle = Math.toRadians((intakeSwivelPos) / ticks_in_degree);

        intakeSwivelPower = pid2 + ff2;

        return intakeSwivelPower;
    }

    //Interface Methods
    @Override
    public void toInit(){
        intakeSlidesRetract();
        intakeSwivelTransfer();
        intakeStopSpin();
    }

    @Override
    public void update(){
        if (usePIDFIntakeSlides) {
            intakeSetSlides(setIntakeSlidesPIDF(intakeSlidesTarget));
        } else {
            if (!(intakeSlidesEnc.getCurrentPosition() > SLIDES_MAX && (intakeSlidesManualPower > 0))) {
                intakeSetSlides(intakeSlidesManualPower);
            } else {
                intakeSetSlides(0);
            }
        }
        if (usePIDFIntakeSwivel) {
            intakeSetSwivel(setIntakeSwivelPIDF(intakeSwivelTarget));
        } else {
            intakeSetSwivel(intakeSwivelManualPower);
        }
        intakeSetSpin(intakeSpinTarget);
    }

}
