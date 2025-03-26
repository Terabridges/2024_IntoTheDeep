package org.firstinspires.ftc.teamcode.Tests;

public class contourProperties {
    private BlockColor color;
    private double distance;
    private double angle;
    private int area;

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

    public BlockColor getColor()
    {
        return color;
    }

    public double getDistance()
    {
        return distance;
    }

    public double angle()
    {
        return angle;
    }

    public int getArea()
    {
        return area;
    }
}
