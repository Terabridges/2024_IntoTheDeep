package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Utility.AbsoluteAnalogEncoder;

public class OuttakeSystem implements Subsystem {

    //Hardware
    public DcMotor outtakeTopVertical;
    public DcMotor outtakeBottomVertical;
    public CRServo outtakeLeftSwivel;
    public CRServo outtakeRightSwivel;
    public Servo outtakeWrist;
    public Servo outtakeClaw;
    public AnalogInput outtakeSwivelAnalog;
    public AbsoluteAnalogEncoder outtakeSwivelEnc;
    public TouchSensor limit;

    //SOFTWARE
    private int servoOffset = 15;
    private int motorOffset = 30;
    public boolean highBasketMode = true;
    public boolean manualOuttake = false;
    public double outtakeSwivelGearRatio = 40.0/30.0;
    public boolean isClawOpen = false;
    private double outtakeSwivelOffset = 180.0;
    public boolean usePIDF = false;
    public boolean useLimitSwitch = false;

    //Positions
    public double CLAW_OPEN = 0.5;
    private double CLAW_CLOSE = 0.175;
    private double WRIST_DOWN = 0.6;
    private double WRIST_TRANSFER = 0.05;
    private double WRIST_UP = 0.94;
    private double WRIST_GRAB = 0.01;
    private double WRIST_LOCK = 0.6; //0.45;
    //LIMIT BACK: 0
    //LIMIT FORWARD: 1
    //SPECIMEN LIMIT: 0.35
    private int OUTTAKE_SWIVEL_UP = 408; //398;
    private int OUTTAKE_SWIVEL_MID = 370;
    private int OUTTAKE_SWIVEL_DOWN = 180;
    private int OUTTAKE_SWIVEL_TRANSFER = 210; //208;
    private int OUTTAKE_SWIVEL_GRAB = 146;
    private int OUTTAKE_SWIVEL_LOCK = 91;
    private int OUTTAKE_SLIDES_HIGH = -3290; //-3320;
    private int OUTTAKE_SLIDES_LOW = -1550; //-1420;
    private int OUTTAKE_SLIDES_DOWN = -40;
    //private int OUTTAKE_SLIDES_REST = -950;
    private int OUTTAKE_SLIDES_REST = -750;
    private int OUTTAKE_SLIDES_GRAB_1 = -40;
    private int OUTTAKE_SLIDES_SCORE_1 = -1655; //-1700;
    private int OUTTAKE_SLIDES_SCORE_2 = -1020;

    private int OUTTAKE_SLIDES_ALMOST_DOWN = -100;

    //Max
    private double OUTTAKE_SLIDES_MAX_POWER = 1.0;
    private double OUTTAKE_SWIVEL_MAX_POWER = 1.0;

    //PIDF
    private double ticks_in_degree = 144.0 / 180.0;

    //Third PID for outtake slides
    public PIDController outtakeSlidesController;
    public static double p3 = 0.008, i3 = 0.001, d3 = 0.0;
    public static double f3 = 0.0;
    public static int outtakeSlidesTarget;
    double outtakeSlidesPos;
    double pid3, targetOuttakeSlidesAngle, ff3, currentOuttakeSlidesAngle, outtakeSlidesPower;

    //Fourth PID for outtake swivel
    public PIDController outtakeSwivelController;
    public static double p4 = 0.003, i4 = 0.001, d4 = 0.00005;
    public static double f4 = -0.02;
    public static int outtakeSwivelTarget;
    double outtakeSwivelPos;
    double pid4, targetOuttakeSwivelAngle, ff4, currentOuttakeSwivelAngle, outtakeSwivelPower;

    //Constructor
    public OuttakeSystem(HardwareMap map) {
        outtakeTopVertical = map.get(DcMotor.class, "outtake_top_vertical");
        outtakeBottomVertical = map.get(DcMotor.class, "outtake_bottom_vertical");
        outtakeLeftSwivel = map.get(CRServo.class, "outtake_left_swivel");
        outtakeRightSwivel = map.get(CRServo.class, "outtake_right_swivel");
        outtakeWrist = map.get(Servo.class, "outtake_wrist");
        outtakeClaw = map.get(Servo.class, "outtake_claw");
        outtakeSwivelAnalog = map.get(AnalogInput.class, "outtake_right_swivel_analog");
        outtakeSwivelEnc = new AbsoluteAnalogEncoder(outtakeSwivelAnalog, 3.3, outtakeSwivelOffset, outtakeSwivelGearRatio);
        limit = map.get(TouchSensor.class, "limit");

        outtakeTopVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeLeftSwivel.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeSlidesController = new PIDController(p3, i3, d3);
        outtakeSwivelController = new PIDController(p4, i4, d4);
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

    public void outtakeSlidesGrab1() {
        outtakeSlidesTarget = OUTTAKE_SLIDES_GRAB_1;
    }

    public void outtakeSlidesScore1() {
        outtakeSlidesTarget = OUTTAKE_SLIDES_SCORE_1;
    }

    public void outtakeSlidesScore2() {
        outtakeSlidesTarget = OUTTAKE_SLIDES_SCORE_2;
    }

    public void outtakeSwivelUp() {
        outtakeSwivelTarget = OUTTAKE_SWIVEL_UP;
    }

    public void outtakeSwivelDown() {
        outtakeSwivelTarget = OUTTAKE_SWIVEL_DOWN;
    }

    public void outtakeSwivelMid() {
        outtakeSwivelTarget = OUTTAKE_SWIVEL_MID;
    }

    public void outtakeSwivelGrab() {
        outtakeSwivelTarget = OUTTAKE_SWIVEL_GRAB;
    }

    public void outtakeSwivelLock() {
        outtakeSwivelTarget = OUTTAKE_SWIVEL_LOCK;
    }

    public void outtakeSwivelTransfer(){outtakeSwivelTarget = OUTTAKE_SWIVEL_TRANSFER;}

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

    public void wristTransfer() {setWrist(WRIST_TRANSFER);}

    public void wristGrab() {setWrist(WRIST_GRAB);}

    public void wristLock() {setWrist(WRIST_LOCK);}

    public void resetSlideEncoders() {
        outtakeBottomVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeBottomVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncodersButton() {
        outtakeBottomVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //isPositions
    public boolean isSlidesDown(){
        return Math.abs(outtakeBottomVertical.getCurrentPosition() - OUTTAKE_SLIDES_DOWN) <= motorOffset;
    }

    public boolean isSlidesAlmostDown(){
        return outtakeBottomVertical.getCurrentPosition() >= (-100);
    }

    public boolean isSlidesRest(){
        return Math.abs(outtakeBottomVertical.getCurrentPosition() - OUTTAKE_SLIDES_REST) <= motorOffset;
    }

    public boolean isSlidesHigh(){
        return Math.abs(outtakeBottomVertical.getCurrentPosition() - (OUTTAKE_SLIDES_HIGH)) <= motorOffset;
    }

    public boolean isSlidesAlmostHigh(){
        return outtakeBottomVertical.getCurrentPosition() <= (OUTTAKE_SLIDES_HIGH+350);
    }

    public boolean isSlidesGrab1(){
        return Math.abs(outtakeBottomVertical.getCurrentPosition() - OUTTAKE_SLIDES_GRAB_1) <= motorOffset;
    }
    public boolean isSlidesScore1(){
        return Math.abs(outtakeBottomVertical.getCurrentPosition() - OUTTAKE_SLIDES_SCORE_1) <= motorOffset;
    }
    public boolean isSlidesScore2(){
        return Math.abs(outtakeBottomVertical.getCurrentPosition() - OUTTAKE_SLIDES_SCORE_2) <= motorOffset;
    }

    public boolean isSlidesLow(){
        return Math.abs(outtakeBottomVertical.getCurrentPosition() - OUTTAKE_SLIDES_LOW) <= motorOffset;
    }

    public boolean isSwivelUp() {
        return Math.abs(outtakeSwivelEnc.getCurrentPosition() - OUTTAKE_SWIVEL_UP) <= servoOffset;
    }
    public boolean isSwivelDown() {
        return Math.abs(outtakeSwivelEnc.getCurrentPosition() - OUTTAKE_SWIVEL_DOWN) <= servoOffset;
    }

    public boolean isSwivelGrab() {
        return Math.abs(outtakeSwivelEnc.getCurrentPosition() - OUTTAKE_SWIVEL_GRAB) <= servoOffset;
    }

    //PIDF
    public double setOuttakeSlidesPIDF(int target) {
        outtakeSlidesController.setPID(p3, i3, d3);
        outtakeSlidesPos = outtakeBottomVertical.getCurrentPosition();
        pid3 = outtakeSlidesController.calculate(outtakeSlidesPos, target);
        targetOuttakeSlidesAngle = target;
        ff3 = (Math.sin(Math.toRadians(targetOuttakeSlidesAngle))) * f3;
        currentOuttakeSlidesAngle = Math.toRadians((outtakeSlidesPos) / ticks_in_degree);

        outtakeSlidesPower = pid3 + ff3;

        return outtakeSlidesPower;
    }

    public double setOuttakeSwivelPIDF(int target) {
        outtakeSwivelController.setPID(p4, i4, d4);
        outtakeSwivelPos = outtakeSwivelEnc.getCurrentPosition();
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
        if (!manualOuttake) {
            outtakeSetSlides(setOuttakeSlidesPIDF(outtakeSlidesTarget));
        }
//        if (usePIDF){
//            outtakeSetSlides(setOuttakeSlidesPIDF(outtakeSlidesTarget));
//        }
        outtakeSetSwivel(setOuttakeSwivelPIDF(outtakeSwivelTarget));

        if (useLimitSwitch && (limit.isPressed() && (Math.abs(outtakeBottomVertical.getCurrentPosition()) > 150))){
            resetEncodersButton();
        }

        //IDK ABOUT THIS
        if(outtakeBottomVertical.getMode().equals(DcMotor.RunMode.STOP_AND_RESET_ENCODER)){
            outtakeBottomVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
