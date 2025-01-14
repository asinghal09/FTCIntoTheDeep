package org.firstinspires.ftc.teamcode.LM4_Jan5th;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous
@Config
public class BadObsAuto extends LinearOpMode {
//NOT THIS ONE
    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmSub slidesSubsystem = new ArmSub(hardwareMap, telemetry);
        Pose2d startPos = new Pose2d(-10, 63.5, Math.toRadians(270));
        drive.setPoseEstimate(startPos);


        TrajectorySequence test = drive.trajectorySequenceBuilder(startPos)
                .splineToConstantHeading(new Vector2d(-11, 35), Math.toRadians(270))
                .addTemporalMarker(0.1, () -> {
                    slidesSubsystem.runArmToPos(900,1);

                })
                .addTemporalMarker(0.25, () -> {
                    slidesSubsystem.setSlides(2500);
                })

                .setReversed(true)
                .addTemporalMarker(1.75,() -> {
                    slidesSubsystem.runArmToPos(1200,0.5);
                })
                .waitSeconds(0.9)
                .splineToConstantHeading(new Vector2d(-50,30), Math.toRadians(270))
                .addTemporalMarker(3.15,() -> {
                    slidesSubsystem.clawOpen();
                })
                .splineToSplineHeading(new Pose2d(-70,-13, Math.toRadians(0)), Math.toRadians(180))
                .addTemporalMarker(6,() -> {

                    slidesSubsystem.setSlides(700);
                })
                .strafeLeft(30)
                .strafeTo(new Vector2d(-90,59))
                //.splineToConstantHeading(new Vector2d(-49,60), Math.toRadians(180))
                .setReversed(false)

                .addTemporalMarker(10,() -> {
                    slidesSubsystem.runArmToPos(150,1);

                })
                .addTemporalMarker(12.5,() -> {
                    slidesSubsystem.clawClose();

                })
                .addTemporalMarker(13,() -> {
                    slidesSubsystem.runArmToPos(700,1);
                    slidesSubsystem.setSlides(2500);

                })
                .waitSeconds(3)
                .splineToSplineHeading(new Pose2d(-12,35,Math.toRadians(270)), Math.toRadians(0))
                .addTemporalMarker(20,() -> {
                    slidesSubsystem.runArmToPos(1200,0.75);

                })
                .build();

        slidesSubsystem.clawClose();

        waitForStart();
        if (!isStopRequested()){
            drive.followTrajectorySequence(test);
        }
    }
}