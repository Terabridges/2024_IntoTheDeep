package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ColorTest", group="Test")
@Config
public class ColorTest extends LinearOpMode {

    public RevColorSensorV3 intakeColorSensor;
    public Servo rightLight;

    NormalizedRGBA colors;

    @Override
    public void runOpMode(){
        intakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intake_color_sensor");
        rightLight = hardwareMap.get(Servo.class, "right_light");

        waitForStart();

        while (opModeIsActive()){
            detectColor();
            setLightColor(getColorVal());
            telemetry.addData("Red", colors.red);
            telemetry.addData("Blue", colors.blue);
            telemetry.addData("Green", colors.green);
            telemetry.update();
        }


    }

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
        if ((colors.red > 0.02 && colors.green > 0.02) && (!(Math.abs(colors.red - colors.green) > 0.04)) && colors.blue < 0.6){
            return "yellow";
        } else if (colors.red > 0.019){
            return "red";
        } else if (colors.blue > 0.015){
            return "blue";
        } else {
            return "none";
        }
    }

    public void detectColor(){
        colors = intakeColorSensor.getNormalizedColors();
    }

    public void setLightColor(String chosenColor) {
        rightLight.setPosition(getColorPWN(chosenColor));
    }
}
