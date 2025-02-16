package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;

import org.firstinspires.ftc.teamcode.RoadRunner05x.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner05x.trajectorysequence.TrajectorySequence;
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

            if (gamepad1.a)
                alignOnce(drive, pipeline, slidesSub);

            if (gamepad1.x)
                slidesSub.clawOpen();
            telemetry.addData("x position: ", pipeline.getXPos());
            telemetry.addData("width" , pipeline.getObjectWidth());
            telemetry.addData("distance", pipeline.getDistance());
            telemetry.addData("X movement ", pipeline.calcXMovement());
            telemetry.addData("y movement", pipeline.calcYMovement());
            telemetry.addData("is Aligned", isAligned);
            telemetry.update();
        }
    }
    public void alignWithUpdates(SampleMecanumDrive drive, AlignmentPipeline pipeline, ArmSub slidesSub){
        Pose2d startPos;
        double xMove = pipeline.calcXMovement();
        double distance = pipeline.getDistance();
        while (xMove > 0.15){
            startPos = drive.getPoseEstimate();
            TrajectorySequence moveRight = drive.trajectorySequenceBuilder(startPos)
                    .strafeRight(2)
                    .build();
            drive.followTrajectorySequence(moveRight);
            xMove = pipeline.calcXMovement();
        }

        while (xMove < -0.15){
            startPos = drive.getPoseEstimate();
            TrajectorySequence moveLeft = drive.trajectorySequenceBuilder(startPos)
                    .strafeLeft(2)
                    .build();
            drive.followTrajectorySequence(moveLeft);
            xMove = pipeline.calcXMovement();
        }

        while(distance > 11){
            startPos = drive.getPoseEstimate();
            TrajectorySequence forward = drive.trajectorySequenceBuilder(startPos)
                    .forward(2)
                    .build();
            drive.followTrajectorySequence(forward);
            distance = pipeline.getDistance();
                    }

        if (Math.abs(xMove) < 0.15 && distance <= 11 ){
            startPos = drive.getPoseEstimate();
            TrajectorySequence forwardAfterLineUp = drive.trajectorySequenceBuilder(startPos)
                    .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(20))))
                    .forward(2)
                    .resetConstraints()
                    .build();
            drive.followTrajectorySequence(forwardAfterLineUp);
            slidesSub.clawClose();
        }


    }


    public void alignOnce(SampleMecanumDrive drive, AlignmentPipeline pipeline, ArmSub slidesSub){
        Pose2d startPos;
        double xMove = pipeline.calcXMovement();
        double distance = pipeline.getDistance();

        if (xMove > 0.15){
            startPos = drive.getPoseEstimate();
            TrajectorySequence moveRight = drive.trajectorySequenceBuilder(startPos)
                    .strafeRight(xMove*5)
                    .build();
            drive.followTrajectorySequence(moveRight);
        }

        if (xMove < -0.15){
            startPos = drive.getPoseEstimate();
            TrajectorySequence moveLeft = drive.trajectorySequenceBuilder(startPos)
                    .strafeLeft(xMove*5)
                    .build();
            drive.followTrajectorySequence(moveLeft);
        }
        distance = pipeline.getDistance();
        if (distance > 11){
            startPos = drive.getPoseEstimate();
            TrajectorySequence forward = drive.trajectorySequenceBuilder(startPos)
                    .forward((distance - 10)/2.54)
                    .build();
            drive.followTrajectorySequence(forward);
        }

        startPos = drive.getPoseEstimate();
        TrajectorySequence forwardAfterLineUp = drive.trajectorySequenceBuilder(startPos)
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(20))))
                .forward(5)
                .resetConstraints()
                .build();
        slidesSub.runArmToPos(400,1);
        drive.followTrajectorySequence(forwardAfterLineUp);
        slidesSub.clawClose();


    }



    public void alignWithSpecimen(SampleMecanumDrive drive, AlignmentPipeline pipeline, ArmSub slidesSub){
        Pose2d startPos;
        boolean isAligned = false;
        long startTime = System.currentTimeMillis();
        while (!isAligned && opModeIsActive()) {
            double xMove = pipeline.calcXMovement();
            double yMove = pipeline.calcYMovement();

            if (System.currentTimeMillis() - startTime > 5000) {  // Timeout after 5 seconds
                telemetry.addData("Alignment Failed", "Timed out!");
                telemetry.update();
                break;
            }

            if (Math.abs(xMove) > 0.15) {
                startPos = drive.getPoseEstimate();
                TrajectorySequence moveX = drive.trajectorySequenceBuilder(startPos)
                        .strafeRight(xMove*5)
                        .build();
                drive.followTrajectorySequence(moveX);
            }
            if (Math.abs(yMove) > 0.15 && Math.abs(xMove) <= 0.15){
                isAligned = true;
                startPos = drive.getPoseEstimate();
                TrajectorySequence forward = drive.trajectorySequenceBuilder(startPos)
                        .forward(yMove)
                        .build();
                drive.followTrajectorySequence(forward);
            }


            if (Math.abs(xMove) <= 0.15 && Math.abs(yMove) <= 0.15) {
                startPos = drive.getPoseEstimate();
                TrajectorySequence forwardAfterLineUp = drive.trajectorySequenceBuilder(startPos)
                        .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(20))))
                        .forward(2)
                        .resetConstraints()
                        .build();
                drive.followTrajectorySequence(forwardAfterLineUp);
                slidesSub.clawClose();
            }
        }
    }
}

