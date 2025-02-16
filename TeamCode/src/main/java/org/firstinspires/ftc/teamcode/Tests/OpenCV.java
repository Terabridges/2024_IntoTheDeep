package org.firstinspires.ftc.teamcode.Tests;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "OpenCV Testing")
public class OpenCV extends LinearOpMode {

    // initializes the width of the camera as well as its x and y direction
    double cX = 0;
    double cY = 0;
    double width = 0;

    // initializes camera and constants for camera resolution
    private OpenCvCamera controlHubCam;
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;

    // will be used to calculate distance
    public static final double objectWidthInRealWorld = 3.5; // this is the width of the sample, change if incorrect
    public static final double focalLength = 728; //replace with the actual length, as I have no idea what it is.

    @Override
    public void runOpMode(){
        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        telemetry.addData("Initialization", "is a success");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Coordinates", "(" + (int)(cX) + "," + (int)(cY) + ")");
            telemetry.addData("Distance in Inches", getDistance(width));
        }
    }
    public void initOpenCV(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId","id", hardwareMap.appContext.getPackageName());

        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(
                WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlockDetectionPipeline());
        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    class YellowBlockDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {

            // Preprocess the frame so that it can detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // finds contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // find largest yellow region
            MatOfPoint largestContour = findLargestContour(contours);

            // checks if yellow contour is detected, otherwise do nothing
            if (largestContour != null){
                telemetry.addData("Object found", "!");
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);

                //calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int)(width) + "pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0,255,0),2);

                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + "inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0,255,0),2);

                // calculate the centroid of the largest contour(outline)
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10()/moments.get_m00();
                cY = moments.get_m01()/moments.get_m00();

                // draw a dot at the centroid
                String label = "(" + (int)(cX) + "," + (int)(cY) + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0,255,0),2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
            }
            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            // IMPORTANT: Change the scalar(hue, saturation, value) values to match the colors.
            // These values are incorrect.
            Scalar lowerYellow = new Scalar(100, 100, 100);
            Scalar upperYellow = new Scalar(180, 255, 255);

            //Isolates a region of area based on intensity of yellow
            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            // Applies the masks
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }
        // helper function to findLargestContour
        private MatOfPoint findLargestContour(List<MatOfPoint> contours){
            double maxArea = 0;
            MatOfPoint largestContour = null;

            // loop through each contour to find the largest area and replace it currently
            for (MatOfPoint contour: contours){
                double area = Imgproc.contourArea(contour);
                if (area < maxArea){
                    maxArea = area;
                    largestContour = contour;
                }
            }
            return largestContour;
        }
        // This helper function calculates the width of the axis-aligned bounding box
        private double calculateWidth(MatOfPoint contour){
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }

    // This helper function does distance calculation: (Real Width * Focal) / Width of Pixels
    public static double getDistance(double width){
        double distance = (objectWidthInRealWorld * focalLength) / width;
        return distance;
    }



}
