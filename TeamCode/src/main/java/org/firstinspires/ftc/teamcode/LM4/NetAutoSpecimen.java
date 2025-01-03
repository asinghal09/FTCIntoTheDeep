package org.firstinspires.ftc.teamcode.LM4;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;


@Autonomous
@Config
public class NetAutoSpecimen extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmSub slidesSubsystem = new ArmSub(hardwareMap, telemetry);
        Pose2d startPos = new Pose2d(10, 63.5, Math.toRadians(270));
        drive.setPoseEstimate(startPos);


        TrajectorySequence test = drive.trajectorySequenceBuilder(startPos)
                .splineToConstantHeading(new Vector2d(11, 35), Math.toRadians(270)) //to sub
                .addTemporalMarker(0.1, () -> {
                    slidesSubsystem.runArmToPos(900, 1);
                })
                .addTemporalMarker(0.25, () -> {
                    slidesSubsystem.setSlides(2500);
                })

                .setReversed(true)
                .addTemporalMarker(1.75, () -> {
                    slidesSubsystem.runArmToPos(1200, 0.5);
                })
                .waitSeconds(0.9)
                .splineToConstantHeading(new Vector2d(80, 33), Math.toRadians(270)) // to sample 1
                .addTemporalMarker(3.15, () -> {
                    slidesSubsystem.clawOpen();
                })
                .addTemporalMarker(4, () -> {
                    slidesSubsystem.setSlides(475);
                })
                .addTemporalMarker(4.5, () -> {
                    slidesSubsystem.runArmToPos(50,1);
                })
                .addTemporalMarker(6, () -> {
                    slidesSubsystem.clawClose();
                })
                .setReversed(true)
                .waitSeconds(1.75)
                .addTemporalMarker(6.5, () -> {

                    slidesSubsystem.runArmToPos(1650,1);
                })
                .addTemporalMarker(7, () -> {
                    slidesSubsystem.setSlides(3900);

                })
                .splineToSplineHeading(new Pose2d(94,60,Math.toRadians(45)),Math.toRadians(45)) //to basket first time
                .addTemporalMarker(9, () -> {
                    slidesSubsystem.clawOpen();
                 })
                .waitSeconds(1.5)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(92,30,Math.toRadians(270)),Math.toRadians(270))//to sample 2
                .addTemporalMarker(11.5, () -> {
                    slidesSubsystem.setSlides(550);
                })
                .addTemporalMarker(12, () -> {
                    slidesSubsystem.runArmToPos(50,1);
                })
                .addTemporalMarker(15, () -> {
                    slidesSubsystem.clawClose();
                })
                .addTemporalMarker(15.5, () -> {

                    slidesSubsystem.runArmToPos(1650,1);
                })
                .addTemporalMarker(16, () -> {
                    slidesSubsystem.setSlides(3900);

                })
                .setReversed(true)
                .waitSeconds(1.75)
                .splineToSplineHeading(new Pose2d(90,56.5,Math.toRadians(45)),Math.toRadians(45)) //to basket 2nd time
                .addTemporalMarker(18, () -> {

                    slidesSubsystem.clawOpen();
                })
                .waitSeconds(2)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(89,30,Math.toRadians(270)),Math.toRadians(270))//to sample 3
                .addTemporalMarker(21, () -> {

                    slidesSubsystem.setSlides(0);
                    slidesSubsystem.runArmToPos(2000,1);
                })
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(23,-25,Math.toRadians(0)),Math.toRadians(180)) // to level 1 ascent
                .build();



        slidesSubsystem.clawClose();

        waitForStart();
        if (!isStopRequested()){
            drive.followTrajectorySequence(test);
        }
    }

}