package org.firstinspires.ftc.teamcode.testing.camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@Autonomous(name = "Rectangle Detector")
public class RectangleDetector extends LinearOpMode {

    private OpenCvWebcam webcam;
    private RectangleDetectionPipeline pipeline;

    @Override
    public void runOpMode() {
        // Initialize webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Attach pipeline
        pipeline = new RectangleDetectionPipeline();
        webcam.setPipeline(pipeline);

        // Open camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                // **Enable streaming to FTC Dashboard**
                FtcDashboard.getInstance().startCameraStream(webcam, 30);
            }

            @Override
            public void onError(int errorCode) {
                //telemetry.addData("Error:", "Camera failed to open");
            }
        });

        // Wait for start
        waitForStart();

        while (opModeIsActive()) {
            telemetry = FtcDashboard.getInstance().getTelemetry();
            double angle = pipeline.getDetectedAngle();
            telemetry.addData("Detected Angle", angle);
            telemetry.update();
        }
    }
}
