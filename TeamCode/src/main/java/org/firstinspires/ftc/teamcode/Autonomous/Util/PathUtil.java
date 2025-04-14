package org.firstinspires.ftc.teamcode.Autonomous.Util;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class PathUtil
{
    public PathChain buildLinearPath(Pose start, Pose end) {
        return new PathBuilder()
                .addPath(bezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }
    public BezierLine bezierLine(Pose start, Pose end)
    {
        return new BezierLine(new Point(start), new Point(end));
    }

    public PathChain buildCurvedPath(Pose start, Pose control, Pose end) {
        return new PathBuilder()
                .addPath(bezierCurve(start, control, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    public PathChain buildCurvedPath(Pose start, Pose control1, Pose control2, Pose end) {
        return new PathBuilder()
                .addPath(bezierCurve(start, control1, control2, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    public BezierCurve bezierCurve(Pose start, Pose control, Pose end)
    {
        return new BezierCurve(new Point(start), new Point(control), new Point(end));
    }

    public BezierCurve bezierCurve(Pose start, Pose control1, Pose control2, Pose end)
    {
        return new BezierCurve(new Point(start), new Point(control1), new Point(control2), new Point(end));
    }
}
