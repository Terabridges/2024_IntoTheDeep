package org.firstinspires.ftc.teamcode.Utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@Autonomous(name="Configure Distance Sensors")
public class ConfigureLaserRangefinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        LaserRangefinder lrf = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, "Laser"));
        telemetry.addData("Pin0", lrf.getPin0Mode());
        telemetry.addData("Pin1", lrf.getPin1Mode());
        telemetry.addData("Distance Mode", lrf.getDistanceMode().name());
        telemetry.addData("Timing [Budget, Period]", java.util.Arrays.toString(lrf.getTiming()));
        telemetry.addData("ROI", java.util.Arrays.toString(lrf.getROI()));
        telemetry.addData("Optical Center", java.util.Arrays.toString(lrf.getOpticalCenter()));
        telemetry.update();
        waitForStart();

        // lrf.setTiming(10, 20);

        lrf.setPin0Analog(0, 4000);
        lrf.setPin1Analog(0, 4000);
        //Will setup analog for all future power-ons of the sensor

    }
}
