package org.firstinspires.ftc.teamcode.Utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Configure Distance Sensor", group = "Tests")
public class DistanceSensorConfigure extends LinearOpMode {

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        LaserRangefinder lrf = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, "Laser"));
        waitForStart();
        while (opModeIsActive()) {
            double distance = lrf.getDistance(DistanceUnit.MM);
            telemetry.addData("Distance", distance);
            telemetry.addData("Status", lrf.getStatus());
            telemetry.update();
        }
    }
}
