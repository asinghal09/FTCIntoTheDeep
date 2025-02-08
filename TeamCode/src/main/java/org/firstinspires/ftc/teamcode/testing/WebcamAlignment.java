package org.firstinspires.ftc.teamcode.testing;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LM5_Jan18.ArmSubOld;
import org.openftc.easyopencv.*;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.QualifierCode.ArmSub;

import java.util.Arrays;


@TeleOp(name = "WebcamAlignment", group = "Testing")
public class WebcamAlignment extends LinearOpMode {
    private OpenCvCamera webcam;

    @Override
    public void runOpMode() {

        ArmSub slidesSub = new ArmSub(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        Pose2d startPos = new Pose2d(45, 63.5, Math.toRadians(270));
        drive.setPoseEstimate(startPos);

        boolean isAligned = true;



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
        slidesSub.setJoint(0.4);
        slidesSub.clawOpen();
        slidesSub.spin(0.61);
        slidesSub.runArmToPos(300,1);

        while (opModeIsActive()) {

            telemetry = FtcDashboard.getInstance().getTelemetry();


            String alignment = pipeline.getAlignmentStatus();
            telemetry.addData("Alignment", alignment);

            // **Example: Use Alignment Data to Adjust Robot Movement**
            if (gamepad1.a) { // Example: Press 'A' to auto-align
                isAligned = false;
                while (!isAligned) {
                    double xMove = pipeline.calcXMovement();
                    double yMove = pipeline.calcYMovement();

                    if (Math.abs(xMove) > 0.25) {
                        startPos = drive.getPoseEstimate();
                        TrajectorySequence moveX = drive.trajectorySequenceBuilder(startPos)
                                .strafeRight(xMove)
                                .build();
                        drive.followTrajectorySequence(moveX);
                    }
                    if (Math.abs(yMove) > 0.25){
                        startPos = drive.getPoseEstimate();
                        TrajectorySequence forward = drive.trajectorySequenceBuilder(startPos)
                                .forward(yMove)
                                .build();
                        drive.followTrajectorySequence(forward);
                    }


                    if (Math.abs(xMove) <= 0.25 && Math.abs(yMove) <= 0.25) {
                        isAligned = true;
                        slidesSub.clawClose();
                    }
                }
            }
            telemetry.addData("x position: ", pipeline.getXPos());
            telemetry.addData("width" , pipeline.getObjectWidth());
            telemetry.addData("distance", pipeline.getDistance());
            telemetry.addData("X movement ", pipeline.calcXMovement());
            telemetry.addData("y movement", pipeline.calcYMovement());
            telemetry.addData("is Aligned", isAligned);
            telemetry.update();
        }
    }
}

