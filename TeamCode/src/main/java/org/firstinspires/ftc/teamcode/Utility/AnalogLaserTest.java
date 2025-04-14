package org.firstinspires.ftc.teamcode.Utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name="Test Distance Sensor over Analog", group = "Tests")
public class AnalogLaserTest extends LinearOpMode {

    private AnalogInput lrf;
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lrf = hardwareMap.get(AnalogInput.class, "Laser");
        waitForStart();
        while (opModeIsActive()) {
            double distance = lrf.getVoltage();
            distance = (distance/3.3) * 4000;
            //distance *= 2.5;
            telemetry.addData("Distance", distance);
            telemetry.update();
        }
    }
}
