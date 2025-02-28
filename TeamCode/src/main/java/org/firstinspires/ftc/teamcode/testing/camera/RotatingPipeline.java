package org.firstinspires.ftc.teamcode.testing.camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@Disabled
@Config
@TeleOp
public class RotatingPipeline extends LinearOpMode {
    OpenCvCamera camera;
    SamplePipeline pipeline;

    Servo spinny, claw, joint;
    DcMotor slides, slidesJoint;

    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    public static double spinnyNormalPos = 0.75;
    public static double spinnyTargetPos = spinnyNormalPos;

    @Override
    public void runOpMode() {
        // Initialize the claw servo
        spinny = hardwareMap.get(Servo.class, "spinny");
        claw = hardwareMap.get(Servo.class, "claw");
        joint = hardwareMap.get(Servo.class, "joint");
        slidesJoint = hardwareMap.get(DcMotorEx.class, "slidesJoint");
        slides = hardwareMap.get(DcMotorEx.class, "slides");

        slidesJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slidesJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesJoint.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int slidesJointTarget = 0;
        slidesJoint.setTargetPosition(slidesJointTarget);
        slidesJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        int slidesTargetPos = 0;
        slides.setTargetPosition(slidesTargetPos);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set the pipeline
        pipeline = new SamplePipeline();
        camera.setPipeline(pipeline);

        // Open the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Error Code: " + errorCode);
                telemetry.update();
            }
        });

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        spinny.setPosition(spinnyTargetPos);
        joint.setPosition(1);


        waitForStart();

        while (opModeIsActive()) {
            // Check if the sample is detected
            if (pipeline.sampleX != -1 && pipeline.sampleY != -1) {
                telemetry.addData("Sample Detected", pipeline.sampleAngle);

                // Align claw if a button is pressed (e.g., "A" button)
                if (gamepad1.a) {
                    spinnyTargetPos = calcServoPosFromAngle(pipeline.sampleAngle);
                    telemetry.addData("Claw aligned at pos", spinnyTargetPos);
                }
            } else {
                telemetry.addData("Sample", "Not Detected");
            }

            if (gamepad1.back)
                spinnyTargetPos += 0.05;
            if (gamepad1.start)
                spinnyTargetPos -= 0.05;
            if(gamepad1.b)
                spinnyTargetPos = spinnyNormalPos;
            if (spinnyTargetPos > 1)
                    spinnyTargetPos = 1;
            else if (spinnyTargetPos < 0)
                spinnyTargetPos = 0;
            spinny.setPosition(spinnyTargetPos);

            telemetry.addData("spinny pos" , spinny.getPosition());

            telemetry.update();
        }

        camera.stopStreaming();
    }

    // Maps a sample angle (-90 to 90) to a servo position (0 to 1)
    private double mapAngleToServo(double angle) {
        // Normalize angle to servo range (0 to 1)
        return SERVO_MIN + ((angle + 90) / 180.0) * (SERVO_MAX - SERVO_MIN);
    }

    public static double calcServoPosFromAngle (double angle){
        double servoPos = 0.75;
        if (angle >= 120){
            servoPos = 1.29 - (0.005 * angle);
        }
        else if (angle >= 60 && angle < 120){
            servoPos = 1.13 - (0.0045 * angle);
        }
        else if ((angle < 60 && angle >= 30) || (angle > 15)){
            servoPos = 0.39 - (0.0035 * angle);
        }
        else if(angle >= 15 && angle < 30){
            servoPos = 0.77 - (0.0035 * angle);
        }

        return servoPos;
    }

    // OpenCV Pipeline
    class SamplePipeline extends OpenCvPipeline {
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
}