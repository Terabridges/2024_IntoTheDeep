package org.firstinspires.ftc.teamcode.Tests;

/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utility.contourProperties;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;
import java.util.TreeMap;


//@Disabled
@TeleOp(name = "OpenCvColor")
public class OpenCVColor extends LinearOpMode {
    // initializes the width of the camera as well as its x and y direction
    private static double widthOfContour = 0;
    private static double heightOfContour = 0;
    private static int blueArea;
    private static int redArea;
    private static int yellowArea;
    private static double yellowContourCount = 0;
    private static double blueContourCount = 0;
    private static double redContourCount = 0;
    private static double smallestYellowDistanceFromContour = 60;
    private static double smallestRedDistanceFromContour = 60;
    private static double smallestBlueDistanceFromContour = 60;
    public double currAngle;
    private TreeMap<Double, contourProperties> contourPropMap = new TreeMap<>();
    private boolean obstructionIsFound = false;
    private contourProperties.BlockColor currColor;

    ArrayList<contourProperties> contourPropsList = new ArrayList<>();

    // initializes camera and constants for camera resolution
    private static final int CAMERA_WIDTH = 320;
    private static final int CAMERA_HEIGHT = 240;

    // will be used to calculate distance
    public static final double objectWidthInRealWorld = 1.5; // this is the width of the sample, change if incorrect
    public static final double focalLength = 2.3; //replace with the actual length, as I have no idea what it is.
    private ColorBlobLocatorProcessor currentContour;

    // Constructor where the current color we are on can be inputted
      public OpenCVColor(contourProperties.BlockColor currColor)
    {
        this.currColor = currColor;
    }

    // defines the colorrange for yellow, blue, and red
    public static final ColorRange YELLOW1 = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar(107, 128, 0),
            new Scalar(255, 170, 120)
    );

    public static final ColorRange BLUE1 = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar(16, 0, 157),
            new Scalar(255, 127, 255)
    );

    public static final ColorRange RED1 = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar(32, 165, 0),
            new Scalar(255, 255, 132)
    );


    @Override
    public void runOpMode() {
        // allows the telemetry to be shown in FTCDashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // created the color blob processors for red, blue,and yellow.
        ColorBlobLocatorProcessor colorLocatorBlue = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(BLUE1)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.95, 0.95, 0.95, -0.95))  // search central 95% of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        ColorBlobLocatorProcessor colorLocatorRed = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(RED1)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.95, 0.95, 0.95, -0.95))  // search central 95% of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        ColorBlobLocatorProcessor colorLocatorYellow = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(YELLOW1)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.95, 0.95, 0.95, -0.95))  // search central 95% of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        // inputted all the colorLocators into a vision portal to be checked by vision.
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(colorLocatorBlue)
                .addProcessor(colorLocatorRed)
                .addProcessor(colorLocatorYellow)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();


        // use to speed up debugging
        //telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        //telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit()) {
            portal.resumeLiveView();
            //telemetry.addData("preview on/off", "... Camera Stream\n");

            // Read the current list
            List<ColorBlobLocatorProcessor.Blob> blobsBlue = colorLocatorBlue.getBlobs();
            List<ColorBlobLocatorProcessor.Blob> blobsRed = colorLocatorRed.getBlobs();
            List<ColorBlobLocatorProcessor.Blob> blobsYellow = colorLocatorYellow.getBlobs();

            // Filter out the external colors that are not blocks by area
            ColorBlobLocatorProcessor.Util.filterByArea(1200, 20000, blobsBlue);  // filter out very small blobs.
            ColorBlobLocatorProcessor.Util.filterByArea(1200, 20000, blobsRed);
            ColorBlobLocatorProcessor.Util.filterByArea(1200, 20000, blobsYellow);

            // checks the number of CONTOURS shown on screen
            yellowContourCount = blobsYellow.size();
            redContourCount = blobsRed.size();
            blueContourCount = blobsBlue.size();

            // these are the values if nothing is detected. Resets all the values in that case
            telemetry.addLine(" Area Density Aspect  Center");
            smallestYellowDistanceFromContour = 60;
            smallestRedDistanceFromContour = 60;
            smallestBlueDistanceFromContour = 60;
            obstructionIsFound = false;
            contourPropMap.clear();

            // Display the size (area) and center location for each Blob.
            for (ColorBlobLocatorProcessor.Blob b : blobsBlue) {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
                widthOfContour = Math.min(boxFit.size.width, boxFit.size.height);
                heightOfContour = Math.min(boxFit.size.width, boxFit.size.height); // locates a portion of the whole contour
                double dist = getDistance(widthOfContour);
                double blueEdgeDistanceFromCenter = (getDistanceFromCenter((boxFit.center.y) - (CAMERA_HEIGHT / 2.0))); // gets dist from center
                double angleFromCenter = angleFromCenter(blueEdgeDistanceFromCenter, dist); // calculates angle
                double centerXOfBlob = boxFit.center.x;
                double centerYOfBlob = boxFit.center.y;
                telemetry.addData("width", widthOfContour); // add all telemetry like width, height, and other things
                telemetry.addData("height", heightOfContour);
                telemetry.addData("distance", dist);
                telemetry.addData("Edge Distance From Center", blueEdgeDistanceFromCenter);
                telemetry.addData("Angle", angleFromCenter);
                blueArea = b.getContourArea();
                contourPropMap.put(dist, new contourProperties(contourProperties.BlockColor.BLUE, dist, angleFromCenter, blueArea)); // add the properties to the treemap
                // if (dist < smallestBlueDistanceFromContour)
                //   smallestBlueDistanceFromContour = dist; // locates the closest color

                telemetry.addData("Blue is detected", "!");

            }

            for (ColorBlobLocatorProcessor.Blob b : blobsRed) {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
                widthOfContour = Math.min(boxFit.size.width, boxFit.size.height);
                heightOfContour = Math.min(boxFit.size.width, boxFit.size.height); // locates a portion of the whole contour
                double dist = getDistance(widthOfContour);
                double edgeDistanceFromCenter = (getDistanceFromCenter((boxFit.center.y) - (CAMERA_HEIGHT / 2.0)));
                double angleFromCenter = angleFromCenter(edgeDistanceFromCenter, dist); // calculates angle
                telemetry.addData("width", widthOfContour); // add all telemetry like width, height, and other things
                telemetry.addData("height", heightOfContour);
                telemetry.addData("distance", dist);
                telemetry.addData("Edge Distance From Center", edgeDistanceFromCenter);
                telemetry.addData("Angle", angleFromCenter);
                redArea = b.getContourArea();
                contourPropMap.put(dist, new contourProperties(contourProperties.BlockColor.RED, dist, angleFromCenter, redArea));

                //if (dist < smallestRedDistanceFromContour) // locates the closest color
                //  smallestRedDistanceFromContour = dist;

                telemetry.addData("Red is detected", "!");
            }

            for (ColorBlobLocatorProcessor.Blob b : blobsYellow) {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
                widthOfContour = Math.min(boxFit.size.width, boxFit.size.height);
                heightOfContour = Math.min(boxFit.size.width, boxFit.size.height); // locates a portion of the whole contour
                double dist = getDistance(widthOfContour);
                double edgeDistanceFromCenter = (getDistanceFromCenter((boxFit.center.y) - (CAMERA_HEIGHT / 2.0)));
                double angleFromCenter = angleFromCenter(edgeDistanceFromCenter, dist); // calculates angle
                telemetry.addData("width", widthOfContour); // add all telemetry like width, height, and other things
                telemetry.addData("height", heightOfContour);
                telemetry.addData("distance", dist);
                telemetry.addData("Edge Distance From Center", edgeDistanceFromCenter);
                telemetry.addData("Angle", angleFromCenter);
                yellowArea = b.getContourArea();
                contourPropMap.put(dist, new contourProperties(contourProperties.BlockColor.YELLOW, dist, angleFromCenter, yellowArea));
                //if (dist != smallestYellowDistanceFromContour) // locates the closest color
                //  smallestYellowDistanceFromContour = dist;
                telemetry.addData("Yellow is detected", "!");
            }

            // print all necessary information for troubleshooting
            telemetry.addData("Number of Red Contours", redContourCount);
            telemetry.addData("Number of Blue Contours", blueContourCount);
            telemetry.addData("Number of Yellow Contours", yellowContourCount);
            telemetry.addData("Smallest Red Distance", smallestRedDistanceFromContour);
            telemetry.addData("Smallest Blue Distance", smallestBlueDistanceFromContour);
            telemetry.addData("Smallest Yellow Distance", smallestYellowDistanceFromContour);
            telemetry.addData("Color to go to ", (decideColorForPickup()) + " Lane: " + determineLane());
            telemetry.update();
            sleep(50);
        }
    }
    // distance calculation; dependent on width
    public static double getDistance(double width) {  //406
        double distance = 18.8 * (objectWidthInRealWorld * 12.0 * (focalLength)) / width;
        return distance;
    }

    // distance center of contour is from the center of the camera
    public static double getDistanceFromCenter(double distFromCenter) {
        return (focalLength * distFromCenter) / 72;
    }
    // uses distance from center in order to calculate angle of center contour in respect to camera
    public static double angleFromCenter(double adjacent, double hypotenuse) {
        double angle = (Math.asin(adjacent / hypotenuse));
        return angle * (180.0 / Math.PI);
    }


    public void logicForPickup(contourProperties prop) {
        int indexOfCurrentYellowContour = contourPropsList.indexOf(prop); // find the closest yellow contour
        for (int index = indexOfCurrentYellowContour - 1; index >= 0; index--) {
            currAngle = contourPropsList.get(indexOfCurrentYellowContour).getAngle(); // get the contour angle
            double currDistance = contourPropsList.get(indexOfCurrentYellowContour).getDistance(); // get the contour distance
            if (contourPropsList.get(index).getColor() != currColor
                    && contourPropsList.get(index).getColor() != contourProperties.BlockColor.YELLOW) {
                if ((Math.abs(currAngle - contourPropsList.get(index).getAngle()) <= 2.50) // checks if the yellow is obstructed
                        && Math.abs(currDistance - contourPropsList.get(index).getDistance()) <= 6.00) {
                    telemetry.addData("Obstruction", "is found");
                    obstructionIsFound = true;
                    break;
                } else {
                    obstructionIsFound = false;
                    break;
                }
            }
        }

    }

    // for this function, acceptable color is also red
    public String decideColorForPickup() {
        for (double distance : contourPropMap.keySet()) {
            contourProperties prop = contourPropMap.get(distance);
            contourPropsList.add(prop); // add the props to the treemap
            int indexOfCurrentYellowContour = contourPropsList.indexOf(prop);

            if (prop != null && prop.getDistance() < 30 &&
                    (prop.getColor() == contourProperties.BlockColor.YELLOW // checks if the contour is red or yellow;
                            || prop.getColor() == currColor)) {
                logicForPickup(prop);
                if (!obstructionIsFound) {
                    return "Go to "
                            + contourPropsList.get(indexOfCurrentYellowContour).getColor()
                            + " at distance : "
                            + contourPropsList.get(indexOfCurrentYellowContour).getDistance()
                            + " and at angle: "
                            + contourPropsList.get(indexOfCurrentYellowContour).getAngle();
                }
            }


            /*
            if (prop != null && prop.getDistance() < 30 && prop.getColor() == contourProperties.BlockColor.RED) {
                logicForPickup(prop);
                if (!obstructionIsFound) {
                    return "Go to "
                            + contourPropsList.get(indexOfCurrentYellowContour).getColor()
                            + " at distance : "
                            + contourPropsList.get(indexOfCurrentYellowContour).getDistance()
                            + "and at angle: "
                            + contourPropsList.get(indexOfCurrentYellowContour).getAngle();
                }
            }
        }*/
        }
        return "No possible block to Pickup from here. Move over. ";
    }

    // code to determine lane the robot should go to during bucket auto.
    // this is dependent on the angle the block is in respect to the camera
    public int determineLane()
    {
        if (currAngle > 0)
        {
            return 1;
        }
        else if (currAngle >= -3 && currAngle < 0)
        {
            return 2;
        }

        else if (currAngle <= -3)
        {
            return 3;
        }
        return 0;
    }
}



