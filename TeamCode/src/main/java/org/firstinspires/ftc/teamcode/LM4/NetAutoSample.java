package org.firstinspires.ftc.teamcode.LM4;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;


@Autonomous
@Config
public class NetAutoSample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmSub slidesSubsystem = new ArmSub(hardwareMap, telemetry);
        Pose2d startPos = new Pose2d(40, 63.5, Math.toRadians(270));
        drive.setPoseEstimate(startPos);


        TrajectorySequence test = drive.trajectorySequenceBuilder(startPos)
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(17))))
                .splineToSplineHeading(new Pose2d(68, 59.5, Math.toRadians(45)), Math.toRadians(45)) //to basket w preload
                .addTemporalMarker(0.1, () -> {
                    slidesSubsystem.runArmToPos(1650, 1);
                })
                .addTemporalMarker(0.3, () -> {
                    slidesSubsystem.setSlides(3900);
                })
                .resetConstraints()
                .addTemporalMarker(2.25, () -> {
                    slidesSubsystem.clawOpen();
                })
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(54.5,30.5,Math.toRadians(270)), Math.toRadians(270)) // to first sample
                .addTemporalMarker(3.25, () -> {
                    slidesSubsystem.setSlides(900);
                }).addTemporalMarker(3.65, () -> {
                    slidesSubsystem.runArmToPos(700,1);
                })
                .addTemporalMarker(4.5, () -> {
                    slidesSubsystem.runArmToPos(100,0.25);
                })
                .waitSeconds(1.15)
                .addTemporalMarker(5.5, () -> {
                    slidesSubsystem.clawClose();
                })
                .splineToSplineHeading(new Pose2d(69.5, 61, Math.toRadians(45)), Math.toRadians(45)) //to basket 1st sample
                .addTemporalMarker(6, () -> {
                    slidesSubsystem.runArmToPos(1675,1);
                })
                .addTemporalMarker(6.5, () -> {

                    slidesSubsystem.setSlides(3900);
                })
                .waitSeconds(1.25)
                .addTemporalMarker(8.7, () -> {
                    slidesSubsystem.clawOpen();
                })
                .splineToSplineHeading(new Pose2d(73,29.75,Math.toRadians(270)), Math.toRadians(270)) // to 2nd sample
                .addTemporalMarker(10, () -> {
                    slidesSubsystem.setSlides(900);
                }).addTemporalMarker(11, () -> {
                    slidesSubsystem.runArmToPos(100,0.8);
                })
                .waitSeconds(1.25)
                .addTemporalMarker(12.5, () -> {
                    slidesSubsystem.clawClose();
                })
                .waitSeconds(0.35)
                .splineToSplineHeading(new Pose2d(72, 58.5, Math.toRadians(45)), Math.toRadians(45)) //to basket 2nd sample
                .addTemporalMarker(13, () -> {
                    slidesSubsystem.runArmToPos(1690,1);
                })
                .addTemporalMarker(14, () -> {

                    slidesSubsystem.setSlides(3950);
                })
                .waitSeconds(3)
                .addTemporalMarker(16, () -> {
                    slidesSubsystem.clawOpen();
                })
                .addTemporalMarker(19, () -> {

                    slidesSubsystem.setSlides(0);
                    slidesSubsystem.runArmToPos(2000,1);
                })
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(23,-15,Math.toRadians(0)),Math.toRadians(180)) // to level 1 ascent
                .waitSeconds(3)
                .build();



        slidesSubsystem.clawClose();


        waitForStart();
        if (!isStopRequested()){
            drive.followTrajectorySequence(test);
        }
    }

}