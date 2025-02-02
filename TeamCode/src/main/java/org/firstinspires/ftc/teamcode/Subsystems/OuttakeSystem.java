package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utility.AbsoluteAnalogEncoder;

public class OuttakeSystem implements Subsystem {

    //Hardware
    public DcMotor outtakeTopVertical;
    public DcMotor outtakeBottomVertical;
    public CRServo outtakeLeftSwivel;
    public CRServo outtakeRightSwivel;
    public Servo outtakeWrist;
    public Servo outtakeClaw;
    public AnalogInput outtakeRightSwivelAnalog;
    public AbsoluteAnalogEncoder outtakeRightSwivelEnc;

    //SOFTWARE
    private int servoOffset = 0;
    private int motorOffset = 0;
    public boolean highBasketMode = true;
    public boolean manualOuttake = false;
    public double outtakeSwivelGearRatio = 40.0/30.0;

    //Positions
    private double CLAW_OPEN = 0.65;
    private double CLAW_CLOSE = 0.27;
    private double WRIST_DOWN = 0.4;
    private double WRIST_UP = 0.3;
    private int OUTTAKE_SWIVEL_UP = 350;
    private int OUTTAKE_SWIVEL_DOWN = 120;
    private int OUTTAKE_SLIDES_HIGH = -3400;
    private int OUTTAKE_SLIDES_LOW = -1600;
    private int OUTTAKE_SLIDES_DOWN = 0;
    private int OUTTAKE_SLIDES_REST = -600;

    //Max
    private double OUTTAKE_SLIDES_MAX_POWER = 1.0;
    private double OUTTAKE_SWIVEL_MAX_POWER = 1.0;

    //PIDF
    private double ticks_in_degree = 144.0 / 180.0;

    //Third PID for outtake slides
    private PIDController outtakeSlidesController;
    public static double p3 = 0.008, i3 = 0.001, d3 = 0.0;
    public static double f3 = 0.0;
    public static int outtakeSlidesTarget;
    double outtakeSlidesPos;
    double pid3, targetOuttakeSlidesAngle, ff3, currentOuttakeSlidesAngle, outtakeSlidesPower;

    //Fourth PID for outtake swivel
    private PIDController outtakeSwivelController;
    public static double p4 = 0.0025, i4 = 0.001, d4 = 0.00005;
    public static double f4 = 0.0;
    public static int outtakeSwivelTarget;
    double outtakeSwivelPos;
    double pid4, targetOuttakeSwivelAngle, ff4, currentOuttakeSwivelAngle, outtakeSwivelPower;

    //Constructor
    public OuttakeSystem(HardwareMap map) {
        outtakeTopVertical = map.get(DcMotor.class, "outtake_bottom_vertical");
        outtakeBottomVertical = map.get(DcMotor.class, "outtake_top_vertical");
        outtakeBottomVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeLeftSwivel = map.get(CRServo.class, "outtake_left_swivel");
        outtakeLeftSwivel.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeRightSwivel = map.get(CRServo.class, "outtake_right_swivel");
        outtakeWrist = map.get(Servo.class, "outtake_wrist");
        outtakeClaw = map.get(Servo.class, "outtake_claw");
        outtakeRightSwivelAnalog = map.get(AnalogInput.class, "outtake_right_swivel_analog");
        outtakeRightSwivelEnc = new AbsoluteAnalogEncoder(outtakeRightSwivelAnalog, 3.3, 0, outtakeSwivelGearRatio);

        outtakeBottomVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeLeftSwivel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //METHODS
    //Core Methods
    public void outtakeSetSlides(double pow) {
        if(pow > OUTTAKE_SLIDES_MAX_POWER) pow = OUTTAKE_SLIDES_MAX_POWER;
        if(pow < -OUTTAKE_SLIDES_MAX_POWER) pow = -OUTTAKE_SLIDES_MAX_POWER;
        outtakeBottomVertical.setPower(pow);
        outtakeTopVertical.setPower(pow);
    }

    public void outtakeSetSwivel(double pow) {
        if(pow > OUTTAKE_SWIVEL_MAX_POWER) pow = OUTTAKE_SWIVEL_MAX_POWER;
        if(pow < -OUTTAKE_SWIVEL_MAX_POWER) pow = -OUTTAKE_SWIVEL_MAX_POWER;
        outtakeLeftSwivel.setPower(pow);
        outtakeRightSwivel.setPower(pow);
    }

    public void setClaw(double pos) {
        outtakeClaw.setPosition(pos);
    }

    public void setWrist(double pos) {
        outtakeWrist.setPosition(pos);
    }

    //Other Methods
    public void outtakeSlidesHigh(){
        outtakeSlidesTarget = OUTTAKE_SLIDES_HIGH;
    }

    public void outtakeSlidesLow(){
        outtakeSlidesTarget = OUTTAKE_SLIDES_LOW;
    }

    public void outtakeSlidesDown(){
        outtakeSlidesTarget = OUTTAKE_SLIDES_DOWN;
    }

    public void outtakeSlidesRest() {
        outtakeSlidesTarget = OUTTAKE_SLIDES_REST;
    }

    public void outtakeSwivelUp() {
        outtakeSwivelTarget = OUTTAKE_SWIVEL_UP;
    }

    public void outtakeSwivelDown() {
        outtakeSwivelTarget = OUTTAKE_SWIVEL_DOWN;
    }

    public void openClaw() {
        setClaw(CLAW_OPEN);
    }

    public void closeClaw() {
        setClaw((CLAW_CLOSE));
    }

    public void wristUp() {
        setWrist(WRIST_UP);
    }

    public void wristDown() {
        setWrist(WRIST_DOWN);
    }

    //isPositions
    public boolean isSlidesDown(){
        return Math.abs(outtakeTopVertical.getCurrentPosition() - OUTTAKE_SLIDES_DOWN) <= motorOffset;
    }

    public boolean isSlidesRest(){
        return Math.abs(outtakeTopVertical.getCurrentPosition() - OUTTAKE_SLIDES_REST) <= motorOffset;
    }

    public boolean isSlidesHigh(){
        return Math.abs(outtakeTopVertical.getCurrentPosition() - OUTTAKE_SLIDES_HIGH) <= motorOffset;
    }

    public boolean isSlidesLow(){
        return Math.abs(outtakeTopVertical.getCurrentPosition() - OUTTAKE_SLIDES_LOW) <= motorOffset;
    }

    public boolean isSwivelUp() {
        return Math.abs(outtakeRightSwivelEnc.getCurrentPosition() - OUTTAKE_SWIVEL_UP) <= servoOffset;
    }
    public boolean isSwivelDown() {
        return Math.abs(outtakeRightSwivelEnc.getCurrentPosition() - OUTTAKE_SWIVEL_DOWN) <= servoOffset;
    }

    //PIDF
    public double setOuttakeSlidesPIDF(int target) {
        outtakeSlidesController.setPID(p3, i3, d3);
        outtakeSlidesPos = outtakeTopVertical.getCurrentPosition();
        pid3 = outtakeSlidesController.calculate(outtakeSlidesPos, target);
        targetOuttakeSlidesAngle = target;
        ff3 = (Math.sin(Math.toRadians(targetOuttakeSlidesAngle))) * f3;
        currentOuttakeSlidesAngle = Math.toRadians((outtakeSlidesPos) / ticks_in_degree);

        outtakeSlidesPower = pid3 + ff3;

        return outtakeSlidesPower;
    }

    public double setOuttakeSwivelPIDF(int target) {
        outtakeSwivelController.setPID(p4, i4, d4);
        outtakeSwivelPos = outtakeRightSwivelEnc.getCurrentPosition();
        pid4 = outtakeSwivelController.calculate(outtakeSwivelPos, target);
        targetOuttakeSwivelAngle = target;
        ff4 = (Math.sin(Math.toRadians(targetOuttakeSwivelAngle))) * f4;
        currentOuttakeSwivelAngle = Math.toRadians((outtakeSwivelPos) / ticks_in_degree);

        outtakeSwivelPower = pid4 + ff4;

        return outtakeSwivelPower;
    }

    //Interface Methods
    @Override
    public void toInit(){
        outtakeSlidesRest();
        outtakeSwivelDown();
        closeClaw();
        wristDown();
    }

    @Override
    public void update(){
        outtakeSetSlides(setOuttakeSlidesPIDF(outtakeSlidesTarget));
        outtakeSetSwivel(setOuttakeSwivelPIDF(outtakeSwivelTarget));
    }

}
