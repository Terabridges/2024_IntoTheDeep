package org.firstinspires.ftc.teamcode.Autonomous.Util.Paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.teamcode.Autonomous.Util.AConstants;
import org.firstinspires.ftc.teamcode.Autonomous.Util.PathUtil;

public class BucketPaths extends PathUtil
{
    Pose startPose = new Pose(AConstants.BOT_CENTER_X, 108+ AConstants.BOT_CENTER_Y, Math.toRadians(0));

    Pose scorePoseP = new Pose(17.8, 135, Math.toRadians(335)); //Preload
    Pose scorePose1 = new Pose(18.25, 135.5, Math.toRadians(351));
    Pose scorePose2 = new Pose(18.25, 135.5, Math.toRadians(348.5));
    Pose scorePose3 = new Pose(15.5, 131.5, Math.toRadians(315));
    Pose scorePose4 = new Pose(15.5, 131.5, Math.toRadians(315)); //From sub

    Pose firstSampleEnd = new Pose(21.5, 134.4, Math.toRadians(335));
    Pose secondSampleEnd = new Pose(20, 135.5, Math.toRadians(352));
    Pose thirdSampleEnd = new Pose(25, 130, Math.toRadians(30));

    Pose lane1 = new Pose(57, 93.25, Math.toRadians(270));
    Pose lane2 = new Pose(63, 93.25, Math.toRadians(270));
    Pose lane3 = new Pose(69, 93.25, Math.toRadians(270));
    Pose lane4 = new Pose(75, 93.25, Math.toRadians(270));
    Pose lane1C = new Pose(57, 108, Math.toRadians(270));
    Pose lane2C = new Pose(63, 108, Math.toRadians(270));
    Pose lane3C = new Pose(69, 108, Math.toRadians(270));
    Pose lane4C = new Pose(75, 108, Math.toRadians(270));
    Pose[] lanes = {lane1, lane2, lane3, lane4};
    Pose[] lanesC = {lane1C, lane2C, lane3C, lane4C};

    Pose placeHolder = new Pose(0, 0, Math.toRadians(0));
    Pose controlPointScore = new Pose(65, 128, Math.toRadians(0));

    Pose[] scoreFrom = {placeHolder, firstSampleEnd, secondSampleEnd, thirdSampleEnd, placeHolder};
    Pose[] scoreTo = {scorePoseP, scorePose1, scorePose2, scorePose3, placeHolder};

    public PathChain goToScore, goToScoreFinal, intakeSample, goToSub1, goToSub2;

    public void buildPathsBucket(int curSample, int selectedLane, int PreSelectedLane)
    {
        goToScore = buildLinearPath(scoreFrom[curSample], scoreTo[curSample]);

        if (curSample > 0)
            intakeSample = buildLinearPath(scoreTo[curSample-1], scoreFrom[curSample]);
        else
            intakeSample = buildLinearPath(scoreTo[curSample], scoreFrom[curSample]);

        goToSub1 = buildLinearPath(scorePose3, lanesC[PreSelectedLane]);
        goToSub2 = buildLinearPath(lanesC[PreSelectedLane], lanes[selectedLane]);

        goToScoreFinal = buildCurvedPath(lanes[selectedLane], controlPointScore, scorePose4);
    }

    public Pose getStartPose()
    {
        return startPose;
    }

    public int getLanesLength()
    {
        return lanes.length;
    }
}
