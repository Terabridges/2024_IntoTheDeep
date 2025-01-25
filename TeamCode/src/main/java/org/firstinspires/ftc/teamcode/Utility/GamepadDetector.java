package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadDetector {

    private OpMode myOpMode = null;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();

    public GamepadDetector(OpMode opmode) {
        myOpMode = opmode;
    }

    public void update() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(myOpMode.gamepad1);
    }

    public boolean aPressed() {
        return (currentGamepad1.a && !previousGamepad1.a);
    }

    public boolean bPressed() {
        return (currentGamepad1.b && !previousGamepad1.b);
    }

    public boolean yPressed() {
        return (currentGamepad1.y && !previousGamepad1.y);
    }
}
