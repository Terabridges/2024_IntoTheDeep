package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SlothTest", group = "Tests")
public class SlothTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Status", "Running2");
            telemetry.update();
        }

    }
}
