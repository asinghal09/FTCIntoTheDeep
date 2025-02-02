package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.Servo;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Core;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class sensor extends LinearOpMode {
    OpenCvCamera camera;
    SamplePipeline pipeline;

    Servo clawServo;

    private static final double SERVO_MIN = 0.0; // Minimum servo position
    private static final double SERVO_MAX = 1.0; // Maximum servo position
    private static final double SERVO_NEUTRAL = 0.5; // Neutral position if needed for initialization

    @Override
    public void runOpMode() {
        // Initialize the claw servo
        clawServo = hardwareMap.get(Servo.class, "claw");
        clawServo.setPosition(SERVO_NEUTRAL); // Set the servo to a neutral position during initialization

        // Initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        // Set the pipeline
        pipeline = new SamplePipeline();
        camera.setPipeline(pipeline);

        // Open the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Error Code: " + errorCode);
                telemetry.update();
            }
        });

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Check if the sample is detected
            if (pipeline.sampleX != -1 && pipeline.sampleY != -1) {
                telemetry.addData("Sample Detected", "X: %.2f, Y: %.2f, Angle: %.2f",
                        pipeline.sampleX, pipeline.sampleY, pipeline.sampleAngle);

                // Align claw if a button is pressed (e.g., "A" button)
                if (gamepad1.a) {
                    double servoPosition = mapAngleToServo(pipeline.sampleAngle);
                    clawServo.setPosition(servoPosition);
                    telemetry.addData("Claw Alignment", "Aligned to %.2f° -> Servo Pos: %.2f",
                            pipeline.sampleAngle, servoPosition);
                }
            } else {
                telemetry.addData("Sample", "Not Detected");
            }

            telemetry.update();
        }

        camera.stopStreaming();
    }

    // Maps a sample angle (-90 to 90) to a servo position (0 to 1)
    private double mapAngleToServo(double angle) {
        // Normalize angle to servo range (0 to 1)
        return SERVO_MIN + ((angle + 90) / 180.0) * (SERVO_MAX - SERVO_MIN);
    }

    // OpenCV Pipeline
    class SamplePipeline extends OpenCvPipeline {
        public volatile double sampleX = -1;
        public volatile double sampleY = -1;
        public volatile double sampleAngle = -1;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Mat mask = new Mat();

            // Convert to HSV color space
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Define the range for detecting a specific color (e.g., red)
            Scalar lowerBound = new Scalar(0, 100, 100);  // Adjust these values
            Scalar upperBound = new Scalar(10, 255, 255); // Adjust these values

            // Create a mask for the color
            Core.inRange(hsv, lowerBound, upperBound, mask);

            // Find contours
            Mat hierarchy = new Mat();
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

            return input;
        }
    }
}