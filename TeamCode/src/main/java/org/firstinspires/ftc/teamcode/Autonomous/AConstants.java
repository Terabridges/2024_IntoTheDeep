package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.pathgen.Point;

public class AConstants {
    public static final double ROBOT_WIDTH = 13.117; //Front to back
    public static final double ROBOT_HEIGHT = 13.413; //Side to side
    public static final double BOT_CENTER_X = ROBOT_WIDTH /2;
    public static final double BOT_CENTER_Y = ROBOT_HEIGHT /2;
    public static final double INTAKE_LENGTH = 19; //Length intake extends from bot center to end of intake in inches

    public static final double DROP_TIME = 0.2;
    public static final double PICKUP_TIME = 0.8;

    public static final double PICKUP_OFFSET = 4.5;

    public static final double STANDARD_POWER = .85;
    public static final double LOW_POWER = .65;

    public static final double SAMPLE_X = 40;
    public static final Point FIRST_SAMPLE = new Point(SAMPLE_X, 121.4);
    public static final Point SECOND_SAMPLE = new Point(SAMPLE_X, 130);
    public static final Point THIRD_SAMPLE = new Point(SAMPLE_X, 126);

    public static final double START_X = SAMPLE_X - INTAKE_LENGTH - PICKUP_OFFSET;
    public static final double END_X = SAMPLE_X - INTAKE_LENGTH;
}
