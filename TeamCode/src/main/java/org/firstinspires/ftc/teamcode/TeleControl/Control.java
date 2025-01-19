package org.firstinspires.ftc.teamcode.TeleControl;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Control {
    void update();

    void addTelemetry(Telemetry t);
}
