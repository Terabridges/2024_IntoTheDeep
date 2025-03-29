package org.firstinspires.ftc.teamcode.Autonomous.Util.Paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.teamcode.Autonomous.Util.AConstants;
import org.firstinspires.ftc.teamcode.Autonomous.Util.PathUtil;

public class SpecimenPaths extends PathUtil
{
    Pose startPose = new Pose(AConstants.BOT_CENTER_X, 48+ AConstants.BOT_CENTER_Y, Math.toRadians(180));

    Pose preloadPose = new Pose(32, 60, Math.toRadians(180));
    Pose preloadPoseb = new Pose(35.595, 60, Math.toRadians(180));
    Pose score1b = new Pose(35.675, 63, Math.toRadians(180));
    Pose score2b = new Pose(35.675, 68, Math.toRadians(180));
    Pose score3b = new Pose(35.675, 73, Math.toRadians(180));
    Pose score1 = new Pose(32, 63, Math.toRadians(180));
    Pose score2 = new Pose(32, 68, Math.toRadians(180));
    Pose score3 = new Pose(32, 73, Math.toRadians(180));

    Pose start1 = new Pose(60, 25.5, Math.toRadians(90));
    Pose end1 = new Pose(20, 25.5, Math.toRadians(90));
    Pose start2 = new Pose(58, 16.5, Math.toRadians(90));
    Pose end2 = new Pose(20, 16.5, Math.toRadians(90));
    Pose start3 = new Pose(58, 7, Math.toRadians(90));
    Pose end3 = new Pose(20, 7, Math.toRadians(90));

    Pose control1 = new Pose(24, 49, Math.toRadians(90));
    Pose control2 = new Pose(45, 35, Math.toRadians(90));
    Pose control3 = new Pose(45, 25, Math.toRadians(90));

    Pose prep = new Pose(20, 24, Math.toRadians(0));
    Pose pick = new Pose(12, 24, Math.toRadians(0));
    Pose pickb = new Pose(6, 24, Math.toRadians(0));

    public PathChain scorePreload, scorePreloadb, goPick, goPickb, goPrep1, goPrep2, goPrep3, goScore1, goScore2, goScore3, goScore1b, goScore2b, goScore3b;
    public PathChain pushSamples1, pushSamples2, pushSamples3, pushSamples4, pushSamples5, pushSamples6, goPark;

    Pose[] score = {preloadPoseb, score1b, score2b, score3b};
    Pose[] push = {start1, end1, start2, end2, start3, end3};
    Pose[] control = {control1, control2, control3};

    PathChain[] goPrep = {goPrep1, goPrep2, goPrep3};

    PathChain[] goScore = {goScore1, goScore2, goScore3};
    PathChain[] goScoreB = {goScore1b, goScore2b, goScore3b};

    public void buildPathsSpecimen()
    {
        scorePreload = buildLinearPath(startPose, preloadPose);
        scorePreloadb = buildLinearPath(preloadPose, preloadPoseb);

        goPick = buildLinearPath(prep, pick);
        goPickb = buildLinearPath(pick, pickb);
        goPrep1 = buildLinearPath(push[1], prep);
        goScore1 = buildLinearPath(pick, score1);
        goScore1b = buildLinearPath(score1, score1b);
        goPrep2 = buildLinearPath(score1, prep);
        goScore2 = buildLinearPath(pick, score2);
        goScore2b = buildLinearPath(score2, score2b);
        goPrep3 = buildLinearPath(score2, prep);
        goPark = buildLinearPath(score2, pick);
        goScore3 = buildLinearPath(pick, score3);
        goScore3b = buildLinearPath(score3, score3b);

        pushSamples1 = buildCurvedPath(score[0], control[0], push[0]);
        pushSamples2 = buildLinearPath(push[0], push[1]);
        pushSamples3 = buildCurvedPath(push[1], control[1], push[2]);
        pushSamples4 = buildLinearPath(push[2], push[3]);
        pushSamples5 = buildCurvedPath(push[3], control[2], push[4]);
        pushSamples6 = buildLinearPath(push[4], push[5]);

//        pushSamples = follower.pathBuilder()
//                .addPath(bezierCurve(score[0], control[0], push[0]))
//                .setLinearHeadingInterpolation(score[0].getHeading(), push[0].getHeading())
//
//                .addPath(bezierLine(push[0], push[1]))
//                .setLinearHeadingInterpolation(push[0].getHeading(), push[1].getHeading())
//                .addPath(bezierCurve(push[1], control[1], push[2]))
//                .setLinearHeadingInterpolation(push[1].getHeading(), push[2].getHeading())
//
//                .addPath(bezierLine(push[2], push[3]))
//                .setLinearHeadingInterpolation(push[2].getHeading(), push[3].getHeading())
//                .addPath(bezierCurve(push[3], control[2], push[4]))
//                .setLinearHeadingInterpolation(push[3].getHeading(), push[4].getHeading())
//
//                .addPath(bezierLine(push[4], push[5]))
//                .setLinearHeadingInterpolation(push[4].getHeading(), push[5].getHeading())
//
//                .build();
    }

    public Pose getStartPose()
    {
        return startPose;
    }

    public PathChain getScore(int i)
    {
        return goScore[i];
    }

    public PathChain getScoreB(int i)
    {
        return goScoreB[i];
    }

    public PathChain getPrep(int i)
    {
        return goPrep[i];
    }
}
