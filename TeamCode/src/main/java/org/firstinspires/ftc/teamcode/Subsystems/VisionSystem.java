package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;
import android.view.contentcapture.DataRemovalRequest;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.teamcode.Utility.DataSampler;
import org.firstinspires.ftc.teamcode.Utility.DataSampler.SamplingMethod; //import enum for sampling styles
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utility.contourProperties;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;
import java.util.TreeMap;

public class VisionSystem implements Subsystem {

    //Hardware
    public RevColorSensorV3 intakeColorSensor;
    public AnalogInput leftBackDistance;
    public AnalogInput rightBackDistance;
    public Servo rightLight;

    //Software
    NormalizedRGBA colors;
    private boolean camInited = false;

    public DataSampler rightDistanceSampling;
    public DataSampler leftDistanceSampling;
    public SamplingMethod samplingMethod = SamplingMethod.AVERAGE;
    public int sampleSize = 10;

    HardwareMap hardwareMap;
    public double leftBackDistVal;
    public double rightBackDistVal;
    public boolean willStopAtObstacle = false;
    public boolean isCloseEnough = false;

    public boolean specimenVisionMode = false;

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
    public boolean runCamera = false;

    ArrayList<contourProperties> contourPropsList = new ArrayList<>();

    // initializes camera and constants for camera resolution
    private static final int CAMERA_WIDTH = 320;
    private static final int CAMERA_HEIGHT = 240;

    // will be used to calculate distance
    public static final double objectWidthInRealWorld = 1.5; // this is the width of the sample, change if incorrect
    public static final double focalLength = 2.3; //replace with the actual length, as I have no idea what it is.

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

    ColorBlobLocatorProcessor colorLocatorBlue;
    ColorBlobLocatorProcessor colorLocatorRed;
    ColorBlobLocatorProcessor colorLocatorYellow;
    VisionPortal portal;

    //Constructor
    public VisionSystem(HardwareMap map) {
        intakeColorSensor = map.get(RevColorSensorV3.class, "intake_color_sensor");
        leftBackDistance = map.get(AnalogInput.class, "left_back_distance");
        rightBackDistance = map.get(AnalogInput.class, "right_back_distance");
        rightLight = map.get(Servo.class, "right_light");

        // created the color blob processors for red, blue,and yellow.
        colorLocatorBlue = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(BLUE1)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.95, 0.95, 0.95, -0.95))  // search central 95% of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        colorLocatorRed = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(RED1)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.95, 0.95, 0.95, -0.95))  // search central 95% of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        colorLocatorYellow = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(YELLOW1)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.95, 0.95, 0.95, -0.95))  // search central 95% of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        // inputted all the colorLocators into a vision portal to be checked by vision.
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(colorLocatorBlue)
                .addProcessor(colorLocatorRed)
                .addProcessor(colorLocatorYellow)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
    }

    //Methods

    public double getColorPWN(String color){
        if (color.equals("red")){
            return 0.279;
        } else if (color.equals("blue")){
            return 0.611;
        } else if (color.equals("yellow")){
            return 0.388;
        } else {
            return 0;
        }
    }

    public String getColorVal(){
        colors = intakeColorSensor.getNormalizedColors();
        if ((colors.red > 0.02 && colors.green > 0.02) && (!(Math.abs(colors.red - colors.green) > 0.04)) && colors.blue < 0.6){
            return "yellow";
        } else if (colors.red > 0.019){
            return "red";
        } else if (colors.blue > 0.015){
            return "blue";
        } else {
            return "none";
        }
    }

    public String getColorValSensitive(){
        colors = intakeColorSensor.getNormalizedColors();
        if ((colors.red > 0.02 && colors.green > 0.02) && (!(Math.abs(colors.red - colors.green) > 0.04)) && colors.blue < 0.6){
            return "yellow";
        } else if (colors.red > 0.014){
            return "red";
        } else if (colors.blue > 0.014){
            return "blue";
        } else {
            return "none";
        }
    }

    public boolean isColor(String color){
        return getColorVal().equals(color);
    }

    public boolean isSomething(){
        return !getColorVal().equals("none");
    }

    public void detectColor(){
        colors = intakeColorSensor.getNormalizedColors();
    }

    public void setLightColor(String chosenColor) {
        rightLight.setPosition(getColorPWN(chosenColor));
    }


    public void getDistances() {
        leftBackDistVal = leftBackDistance.getVoltage();
        leftBackDistVal = (leftBackDistVal/3.3) * 4000;

        rightBackDistVal = rightBackDistance.getVoltage();
        rightBackDistVal = (rightBackDistVal/3.3) * 4000;

        rightDistanceSampling.updateData(rightBackDistVal);
        leftDistanceSampling.updateData(leftBackDistVal);

        rightBackDistVal = rightDistanceSampling.calculateData();
        leftBackDistVal = leftDistanceSampling.calculateData();

    }

    public void switchVisionMode() {
        specimenVisionMode = !specimenVisionMode;
    }

    public boolean isClose() {
        return ((leftBackDistVal <= 143 && leftBackDistVal >= 130) || (rightBackDistVal <= 143 && rightBackDistVal >= 130));
        // NOT ACCURATE
        // WILL FIX WHEN TESTING
    }

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
            if (contourPropsList.get(index).getColor() != contourProperties.BlockColor.RED
                    && contourPropsList.get(index).getColor() != contourProperties.BlockColor.YELLOW) {
                if ((Math.abs(currAngle - contourPropsList.get(index).getAngle()) <= 2.50) // checks if the yellow is obstructed
                        && Math.abs(currDistance - contourPropsList.get(index).getDistance()) <= 6.00) {
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
                            || prop.getColor() == contourProperties.BlockColor.RED)) {
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


    //Interface Methods
    @Override
    public void toInit() {
        willStopAtObstacle = false;

        rightDistanceSampling = new DataSampler(samplingMethod, sampleSize);
        leftDistanceSampling = new DataSampler(samplingMethod, sampleSize);

    }

    @Override
    public void update() {
        detectColor();
        if (!specimenVisionMode) {
            setLightColor(getColorVal());
        }
        getDistances();

        if (specimenVisionMode) {
            if (isClose()) {
                rightLight.setPosition(0.444);
            } else {
                rightLight.setPosition(0);
            }

        }

        //Camera stuff
        if (runCamera) {
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
            smallestYellowDistanceFromContour = 60;
            smallestRedDistanceFromContour = 60;
            smallestBlueDistanceFromContour = 60;
            obstructionIsFound = false;
            contourPropMap.clear();

            // Display the size (area) and center location for each Blob.
            for (ColorBlobLocatorProcessor.Blob b : blobsBlue) {
                RotatedRect boxFit = b.getBoxFit();
                widthOfContour = Math.min(boxFit.size.width, boxFit.size.height);
                heightOfContour = Math.min(boxFit.size.width, boxFit.size.height); // locates a portion of the whole contour
                double dist = getDistance(widthOfContour);
                double blueEdgeDistanceFromCenter = (getDistanceFromCenter((boxFit.center.y) - (CAMERA_HEIGHT / 2.0))); // gets dist from center
                double angleFromCenter = angleFromCenter(blueEdgeDistanceFromCenter, dist); // calculates angle
                blueArea = b.getContourArea();
                contourPropMap.put(dist, new contourProperties(contourProperties.BlockColor.BLUE, dist, angleFromCenter, blueArea)); // add the properties to the treemap
                // if (dist < smallestBlueDistanceFromContour)
                //   smallestBlueDistanceFromContour = dist; // locates the closest color


            }

            for (ColorBlobLocatorProcessor.Blob b : blobsRed) {
                RotatedRect boxFit = b.getBoxFit();
                widthOfContour = Math.min(boxFit.size.width, boxFit.size.height);
                heightOfContour = Math.min(boxFit.size.width, boxFit.size.height); // locates a portion of the whole contour
                double dist = getDistance(widthOfContour);
                double edgeDistanceFromCenter = (getDistanceFromCenter((boxFit.center.y) - (CAMERA_HEIGHT / 2.0)));
                double angleFromCenter = angleFromCenter(edgeDistanceFromCenter, dist); // calculates angle
                redArea = b.getContourArea();
                contourPropMap.put(dist, new contourProperties(contourProperties.BlockColor.RED, dist, angleFromCenter, redArea));
                //if (dist < smallestRedDistanceFromContour) // locates the closest color
                //  smallestRedDistanceFromContour = dist;

            }

            for (ColorBlobLocatorProcessor.Blob b : blobsYellow) {
                RotatedRect boxFit = b.getBoxFit();
                widthOfContour = Math.min(boxFit.size.width, boxFit.size.height);
                heightOfContour = Math.min(boxFit.size.width, boxFit.size.height); // locates a portion of the whole contour
                double dist = getDistance(widthOfContour);
                double edgeDistanceFromCenter = (getDistanceFromCenter((boxFit.center.y) - (CAMERA_HEIGHT / 2.0)));
                double angleFromCenter = angleFromCenter(edgeDistanceFromCenter, dist); // calculates angle
                yellowArea = b.getContourArea();
                contourPropMap.put(dist, new contourProperties(contourProperties.BlockColor.YELLOW, dist, angleFromCenter, yellowArea));
                //if (dist != smallestYellowDistanceFromContour) // locates the closest color
                //  smallestYellowDistanceFromContour = dist;

            }
        }
    }
}
