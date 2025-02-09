package org.firstinspires.ftc.teamcode.testing;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

public class AlignmentPipeline extends OpenCvPipeline {
    private Mat hsvMat = new Mat();
    Scalar lowerRedLower = new Scalar(0, 120, 100);    // Lower range (0-10)
    Scalar lowerRedUpper = new Scalar(10, 255, 255);
    Scalar upperRedLower = new Scalar(170, 120, 100);  // Upper range (170-180)
    Scalar upperRedUpper = new Scalar(180, 255, 255);
    private Scalar lowerBlue = new Scalar (100,100,100);
    private Scalar upperBlue = new Scalar (130,255,255);
    private String alignmentStatus = "Unknown";
    private double xPos = -1;
    private double distance = -1;
    private double xMove = 0;
    public double yMove = 0;
    private double targetX = 320;
    private double targetY = 15;

    private static final double SAMPLE_WIDTH = 3.81;
    private static final double FOCAL_LENGTH = 1008;//calculated by doing (216 pixels * 17.78 cm)/3.81cm

    private double objectWidth = -1;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Mat lowerRedMask = new Mat();
        Mat upperRedMask = new Mat();
        Mat redMask = new Mat();
        Mat blueMask = new Mat();
        Mat combinedMask = new Mat();
        Core.inRange(hsvMat, lowerRedLower, lowerRedUpper, lowerRedMask);
        Core.inRange(hsvMat, upperRedLower, upperRedUpper, upperRedMask);
        Core.inRange(hsvMat, lowerBlue, upperBlue, blueMask);

        // Combine both masks (red and blue)
        Core.addWeighted(lowerRedMask, 1.0, upperRedMask, 1.0,0.0, redMask);
        Core.addWeighted(blueMask, 1.0,redMask, 1.0,0.0,combinedMask);


        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find largest object
        double maxArea = 0;
        Rect bestRect = null;
        for (MatOfPoint contour : contours) {
            Rect rect = Imgproc.boundingRect(contour);
            double area = rect.width * rect.height;
            if (area > maxArea) {
                maxArea = area;
                bestRect = rect;
            }
        }

        // If object found, calculate X position and distance
        if (bestRect != null) {
            objectWidth = bestRect.width;
            xPos = bestRect.x + bestRect.width / 2.0; // X center of object
            distance = (SAMPLE_WIDTH * FOCAL_LENGTH) / bestRect.width; // Distance calculation

            // Draw rectangle on screen
            Imgproc.rectangle(input, bestRect, new Scalar(0, 255, 0), 2);
        } else {
            xPos = -1;
            distance = -1;
        }


        /*
        // Find the largest red blob (game element)
        Moments moments = Imgproc.moments(hsvMat);
        if (moments.m00 > 0) {
            int x = (int) (moments.m10 / moments.m00); // X position of detected element
            xPos = x;
            int frameTargetX = 270;

            if (x < frameTargetX - 20) {
                //telemetry.addData("Alignment", "Move Left");
                alignmentStatus = "Move Left";
            } else if (x > frameTargetX + 20) {
                //telemetry.addData("Alignment", "Move Right");
                alignmentStatus = "Move Right";
            } else {
                //telemetry.addData("Alignment", "Aligned!");
                alignmentStatus = "Aligned";
            }

        } else{
            alignmentStatus = "Not Found";
        }

         */

        //return hsvMat; // Return the processed frame

        return input;

    }

    public double calcXMovement(){  //1 in = 180 px
        xMove = xPos - targetX; //need to move by this many px, positive is right negative is left
        xMove /= 180;       // convert px to inches
        return xMove;
    }

    public double calcYMovement(){  //1 in = 180 px
        yMove = distance - targetY; //need to move forward by this much
        yMove /= 2.54;       // convert cm to inches
        return yMove;
    }


    public double getXPos (){
        return xPos;
    }
    public double getDistance(){
        return distance;
    }
    public double getObjectWidth(){
        return objectWidth;
    }

    public String getAlignmentStatus() {
        return alignmentStatus;
    }

    public void update (){

    }
}
