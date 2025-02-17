package org.firstinspires.ftc.teamcode.testing.camera;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RectangleDetectionPipeline extends OpenCvPipeline {

    // Define lower and upper bounds for blue
    Scalar lowerBlue = new Scalar (100,100,100);
    Scalar upperBlue = new Scalar (130,255,255);


    private double detectedAngle = 0;  // Store the detected angle

    @Override
    public Mat processFrame(Mat input) {
        // Convert to HSV
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);


        // Create a binary mask where blue colors are white, everything else is black
        Mat mask = new Mat();
        Core.inRange(hsv, lowerBlue, upperBlue, mask);

        // Find edges on the masked image
        Mat edges = new Mat();
        Imgproc.Canny(mask, edges, 50, 150);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Loop through contours to find rectangles
        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approxCurve, 0.02 * Imgproc.arcLength(contour2f, true), true);

            if (approxCurve.total() == 4) {
                RotatedRect rect = Imgproc.minAreaRect(approxCurve);
                Point[] vertices = new Point[4];
                rect.points(vertices);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 2);
                }
                detectedAngle = rect.angle;
            }
        }

        return input;  // Return processed frame with rectangles highlighted
    }


    public double getDetectedAngle() {
        return detectedAngle;
    }
}