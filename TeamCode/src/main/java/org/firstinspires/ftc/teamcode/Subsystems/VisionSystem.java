package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class VisionSystem implements Subsystem {

    //Hardware
    public RevColorSensorV3 intakeColorSensor;
//    public I2cDevice leftBackDistance;
//    public I2cDevice rightBackDistance;
//    public TouchSensor magLimitSwitch;
    public Servo rightLight;

    //Software
    NormalizedRGBA colors;
    private boolean camInited = false;

    public VisionPortal vp;
    public VisionPortal.Builder vpBuilder;
    public List<AprilTagDetection> detections;
    public List<VisionProcessor> processors;
    HardwareMap hardwareMap;
    public double leftBackDistanceVal;
    public double rightBackDistanceVal;


    //Constructor
    public VisionSystem(HardwareMap map) {
        intakeColorSensor = map.get(RevColorSensorV3.class, "intake_color_sensor");
//        leftBackDistance = map.get(I2cDevice.class, "left_back_distance");
//        rightBackDistance = map.get(I2cDevice.class, "right_back_distance");
//        magLimitSwitch = map.get(TouchSensor.class, "limit_switch");
        rightLight = map.get(Servo.class, "right_light");
    }

    //Methods
//    public void getDistances(){
//        leftBackDistanceVal = (leftBackDistance.getVoltage()*48.7)-4.9;
//        rightBackDistanceVal = (rightBackDistance.getVoltage()*48.7)-4.9;
//    }

    public double getColorPWN(String color){
        if (color.equals("red")){
            return 0.279;
        } else if (color.equals("blue")){
            return 0.611;
        } else if (color.equals("yellow")){
            return 0.388;
        } else {
            return 0;
        }
    }

    public String getColorVal(){
        if (colors.red > 0.07 && colors.green > 0.07){
            return "yellow";
        } else if (colors.red > 0.07){
            return "red";
        } else if (colors.blue > 0.05){
            return "blue";
        } else {
            return "none";
        }
    }

    public boolean isColor(String color){
        return getColorVal().equals(color);
    }

    public boolean isSomething(){
        return !getColorVal().equals("none");
    }

    public void detectColor(){
        colors = intakeColorSensor.getNormalizedColors();
    }

    public void setLightColor(String chosenColor) {
        rightLight.setPosition(getColorPWN(chosenColor));
    }

    public void setLookForBasket(){

    }

    //Interface Methods
    @Override
    public void toInit() {
        if (!camInited) {
            initProcessors();
            vp = vpBuilder.build();

            vp.resumeStreaming();
            camInited = true;
        }
    }

    private void initProcessors() {
        if(camInited) return;
        for (VisionProcessor processor : processors) {
            vpBuilder.addProcessors(processor);
        }
    }

    private void addProcessors(VisionProcessor... processors) {
        if (camInited) return;
        for (VisionProcessor processor : processors) {
            vpBuilder.addProcessor(processor);
        }
    }

    public void addAprilTag() {
        addProcessors(new AprilTagProcessor.Builder().setDrawAxes(true).build());
    }

    private VisionPortal.Builder vpBuilderFront() {

        // Create a VisionPortal builder
        return new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "front_camera"));

    }

    private VisionPortal.Builder vpBuilderBack() {

        // Create a VisionPortal builder
        return new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "back_camera"));

    }



    @Override
    public void update(){
        detectColor();
        setLightColor(getColorVal());
        //getDistances();
    }
}
