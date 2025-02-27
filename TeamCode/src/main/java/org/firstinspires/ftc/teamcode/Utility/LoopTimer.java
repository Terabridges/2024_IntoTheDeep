package org.firstinspires.ftc.teamcode.Utility;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class LoopTimer {

    private ElapsedTime loopTimer;
    private double pastLoopTime;
    private double currentLoopTime;

    public LoopTimer() {
        loopTimer = new ElapsedTime();
        loopTimer.reset();

        pastLoopTime = 0;
        currentLoopTime = 0;
    }

    public void updateTime(Telemetry telemetry) {
        double currentTime = loopTimer.milliseconds();

        if (currentTime > 10000) {
            loopTimer.reset();
        }

        pastLoopTime = currentLoopTime;
        currentLoopTime = currentTime;

        if (currentLoopTime - pastLoopTime > 0) {
            telemetry.addData("Loop Time", currentLoopTime - pastLoopTime);
        }

    }

}
