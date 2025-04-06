package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoPulseWidthCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@Disabled
@TeleOp(name = "ServoTester", group = "Tests")
public class ServoTester extends LinearOpMode {

    // Button edge detection states
    boolean prevDpadUp = false;
    boolean prevDpadDown = false;
    boolean prevDpadLeft = false;
    boolean prevDpadRight = false;
    boolean prevA = false;
    boolean prevB = false;
    boolean prevY = false;
    boolean prevRB = false;
    boolean prevLB = false;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        if (hubs.size() < 1) {
            telemetry.addData("ERROR", "No REV hubs found");
            telemetry.update();
            return;
        }

        int hubIndex = 0;
        int port = 0;
        double position = 0.5;      // Normalized position [0.0 - 1.0]
        double increment = 0.05;    // Step size

        waitForStart();

        while (opModeIsActive()) {
            // --- Edge Detection ---
            boolean dpadUpPressed = gamepad1.dpad_up && !prevDpadUp;
            boolean dpadDownPressed = gamepad1.dpad_down && !prevDpadDown;
            boolean dpadLeftPressed = gamepad1.dpad_left && !prevDpadLeft;
            boolean dpadRightPressed = gamepad1.dpad_right && !prevDpadRight;
            boolean aPressed = gamepad1.a && !prevA;
            boolean bPressed = gamepad1.b && !prevB;
            boolean yPressed = gamepad1.y && !prevY;
            boolean rbPressed = gamepad1.right_bumper && !prevRB;
            boolean lbPressed = gamepad1.left_bumper && !prevLB;

            // --- Hub & Port Selection ---
            if (dpadUpPressed) hubIndex = (hubIndex + 1) % hubs.size();
            if (dpadDownPressed) hubIndex = (hubIndex - 1 + hubs.size()) % hubs.size();
            if (dpadRightPressed) port = (port + 1) % 6;
            if (dpadLeftPressed) port = (port - 1 + 6) % 6;

            // --- Position & Increment Control ---
            if (rbPressed) position = Math.min(position + increment, 1.0);
            if (lbPressed) position = Math.max(position - increment, 0.0);

            if (aPressed) increment = 0.01;
            if (bPressed) increment = 0.05;
            if (yPressed) increment = 0.10;

            // --- Convert to PWM ---
            int pulseWidth = 500 + (int)(position * 2000); // 500–2500 µs

            // --- Send PWM Command ---
            try {
                LynxSetServoPulseWidthCommand cmd =
                        new LynxSetServoPulseWidthCommand(hubs.get(hubIndex), port, pulseWidth);
                cmd.send();
            } catch (Exception e) {
                telemetry.addData("ERROR", "Send failed: %s", e.getMessage());
            }

            // --- Telemetry ---
            telemetry.addLine("== Servo Tester ==");
            telemetry.addData("Selected Hub Index", hubIndex);
            telemetry.addData("Servo Port", port);
            telemetry.addData("Position (Normalized)", "%.3f", position);
            telemetry.addData("PWM Pulse Width (µs)", pulseWidth);
            telemetry.addData("Increment", "%.3f", increment);

            telemetry.addLine("\n== Servo Bus Currents ==");
            for (int i = 0; i < hubs.size(); i++) {
                LynxModule hub = hubs.get(i);
                double current = getServoBusCurrent(hub);
                telemetry.addLine(String.format("Hub %d [%s]", i, hub.getSerialNumber()));
                telemetry.addData(" - Servo Bus Current (A)", current >= 0 ? String.format("%.2f", current) : "ERROR");
            }

            telemetry.addLine("\nControls: DPad = hub/port | Bumpers = pos | A/B/Y = step");
            telemetry.update();

            // --- Save Button States ---
            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;
            prevDpadLeft = gamepad1.dpad_left;
            prevDpadRight = gamepad1.dpad_right;
            prevA = gamepad1.a;
            prevB = gamepad1.b;
            prevY = gamepad1.y;
            prevRB = gamepad1.right_bumper;
            prevLB = gamepad1.left_bumper;

            sleep(50);
        }
    }

    private double getServoBusCurrent(LynxModule hub) {
        try {
            LynxGetADCCommand cmd = new LynxGetADCCommand(
                    hub,
                    LynxGetADCCommand.Channel.SERVO_CURRENT,
                    LynxGetADCCommand.Mode.ENGINEERING);
            LynxGetADCResponse response = cmd.sendReceive();
            return response.getValue() / 1000.0; // mA to A
        } catch (InterruptedException | RuntimeException | LynxNackException e) {
            return -1;
        }
    }
}
