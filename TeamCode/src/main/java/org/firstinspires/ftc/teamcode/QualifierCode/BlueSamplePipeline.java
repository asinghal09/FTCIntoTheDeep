package org.firstinspires.ftc.teamcode.QualifierCode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BlueSamplePipeline extends OpenCvPipeline {
    private Mat hsv = new Mat(); // Declare and initialize outside processFrame
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();

    public volatile double sampleX = -1;
    public volatile double sampleY = -1;
    public volatile double sampleAngle = -1;

    @Override
    public Mat processFrame(Mat input) {
        // Reuse Mat objects instead of creating new ones
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Define the range for detecting a specific color (e.g., red)
        Scalar lowerBound = new Scalar(100, 100, 100);
        Scalar upperBound = new Scalar(130, 255, 255);

        // Create a mask for the color
        Core.inRange(hsv, lowerBound, upperBound, mask);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour
        double maxArea = 0;
        RotatedRect largestRect = null;

        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rect = Imgproc.minAreaRect(contour2f);
            double area = Imgproc.contourArea(contour);

            if (area > maxArea) {
                maxArea = area;
                largestRect = rect;
            }

            contour2f.release(); // Release memory for temporary MatOfPoint2f
        }

        if (largestRect != null) {
            // Calculate the angle and position of the detected sample
            sampleAngle = largestRect.angle;

            // Normalize angle to -90 to 90 degrees
            if (largestRect.size.width < largestRect.size.height) {
                sampleAngle += 90;
            }

            // Get the center of the detected sample
            sampleX = largestRect.center.x;
            sampleY = largestRect.center.y;

            // Draw the rectangle and center on the frame
            Point[] boxPoints = new Point[4];
            largestRect.points(boxPoints);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(0, 255, 0), 2);
            }
            Imgproc.circle(input, largestRect.center, 5, new Scalar(255, 0, 0), -1);
        } else {
            // No sample detected; reset values
            sampleX = -1;
            sampleY = -1;
            sampleAngle = -1;
        }

        return input; // Return the modified input frame
    }

    @Override
    public void finalize() {
        // Release resources when the pipeline is destroyed
        hsv.release();
        mask.release();
        hierarchy.release();
    }
}
