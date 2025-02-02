package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class ConfigureLaserRangefinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LaserRangefinder lrf = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, "Laser"));
        telemetry.addData("Pin0", lrf.getPin0Mode());
        telemetry.addData("Pin1", lrf.getPin1Mode());
        telemetry.addData("Distance Mode", lrf.getDistanceMode().name());
        telemetry.addData("Timing [Budget, Period]", java.util.Arrays.toString(lrf.getTiming()));
        telemetry.addData("ROI", java.util.Arrays.toString(lrf.getROI()));
        telemetry.addData("Optical Center", java.util.Arrays.toString(lrf.getOpticalCenter()));
        telemetry.update();
        waitForStart();
        /* <configuration code> */
    }
}
