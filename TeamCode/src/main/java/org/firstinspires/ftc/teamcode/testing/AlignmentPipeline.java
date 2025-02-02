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
    private Scalar lowerRed = new Scalar(0, 100, 100);
    private Scalar upperRed = new Scalar(10, 255, 255);
    private String alignmentStatus = "Unknown";

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, lowerRed, upperRed, hsvMat);

        // Find the largest red blob (game element)
        Moments moments = Imgproc.moments(hsvMat);
        if (moments.m00 > 0) {
            int x = (int) (moments.m10 / moments.m00); // X position of detected element
            int frameCenter = input.width() / 2;

            if (x < frameCenter - 20) {
                //telemetry.addData("Alignment", "Move Left");
                alignmentStatus = "Move Left";
            } else if (x > frameCenter + 20) {
                //telemetry.addData("Alignment", "Move Right");
                alignmentStatus = "Move Right";
            } else {
                //telemetry.addData("Alignment", "Aligned!");
                alignmentStatus = "Aligned";
            }
        }

        return hsvMat; // Return the processed frame
    }


    public String getAlignmentStatus() {
        return alignmentStatus;
    }
}
