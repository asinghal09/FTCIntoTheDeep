package org.firstinspires.ftc.teamcode.testing;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;


@TeleOp(name = "WebcamAlignment", group = "Testing")
public class WebcamAlignment extends LinearOpMode {
    private OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPos = new Pose2d(45, 63.5, Math.toRadians(270));
        drive.setPoseEstimate(startPos);


        TrajectorySequence left = drive.trajectorySequenceBuilder(startPos)
                .strafeLeft(2)
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPos)
                .strafeRight(2)
                .build();

        TrajectorySequence back = drive.trajectorySequenceBuilder(startPos)
                .back(2)
                .build();



        //Attach the custom pipeline
        AlignmentPipeline pipeline = new AlignmentPipeline();
        webcam.setPipeline(pipeline);

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

        waitForStart();



        while (opModeIsActive()) {


            String alignment = pipeline.getAlignmentStatus();
            //telemetry.addData("Alignment", alignment);

            // **Example: Use Alignment Data to Adjust Robot Movement**
            if (gamepad1.a) { // Example: Press 'A' to auto-align
                if (alignment.equals("Move Left")) {
                    // Move robot left
                    drive.followTrajectorySequence(left);
                } else if (alignment.equals("Move Right")) {
                    // Move robot right
                    drive.followTrajectorySequence(right);
                } else {
                    // Stop movement
                    drive.followTrajectorySequence(back);
                }
            }
            telemetry.update();
        }
    }
}

