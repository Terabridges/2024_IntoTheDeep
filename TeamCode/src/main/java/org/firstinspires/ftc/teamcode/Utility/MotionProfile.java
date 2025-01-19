package org.firstinspires.ftc.teamcode.Utility;

public class MotionProfile {
    public double goalPosition;
    public double goalVelocity;
    public double goalAcceleration;

    public MotionProfile(double gp, double gv, double ga) {
        goalPosition = gp;
        goalVelocity = gv;
        goalAcceleration = ga;
    }

    public static MotionProfile trapMotion(double t, double maxAccel, double maxVel, double startPos, double endPos) {
        double gp, gv, ga;
        if (endPos < startPos) {
            maxVel = -maxVel;
            maxAccel = -maxAccel;
        }
        double accelTime = maxVel / maxAccel;
        double totalTime, cruiseTime;

        if (Math.abs(accelTime * maxVel) > Math.abs((endPos - startPos))) {
            accelTime = Math.sqrt(Math.abs((endPos - startPos) / maxAccel));
            totalTime = 2 * accelTime;
            cruiseTime = 0;
        } else {
            totalTime = 2 * accelTime + Math.abs((Math.abs(endPos - startPos) - Math.abs(accelTime * maxVel)) / maxVel);
            cruiseTime = totalTime - 2 * accelTime;

        }
        cruiseTime = Math.max(cruiseTime, 0);

        if (t < accelTime) {
            ga = maxAccel;
            gv = t * ga;
            gp = t * gv / 2;
        }
        else if (t < accelTime + cruiseTime) {
            ga = 0;
            gv = maxVel;
            gp = accelTime * maxVel / 2 + (t - accelTime) * maxVel;
        } else if (t < totalTime) {
            ga = -maxAccel;
            gv = (totalTime - t) * -ga;
            gp = (totalTime + cruiseTime) * (cruiseTime != 0 ? maxVel : accelTime * maxAccel) / 2 - (totalTime - t) * gv / 2;
        } else {
            gp = endPos - startPos;
            gv = 0;
            ga = 0;
        }

        gp += startPos;

        return new MotionProfile(gp, gv, ga);
    }
}
