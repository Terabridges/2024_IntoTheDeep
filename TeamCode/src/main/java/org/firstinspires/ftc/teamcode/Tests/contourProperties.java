package org.firstinspires.ftc.teamcode.Tests;

public class contourProperties {
    public BlockColor color;
    public double distance;
    public double angle;
    public int area;

    enum BlockColor
    {
        RED,
        YELLOW,
        BLUE
    }

    public contourProperties(BlockColor color, double distance,  double angle, int area)
    {
        this.color = color;
        this.distance = distance;
        this.angle = angle;
        this.area = area;
    }
}
