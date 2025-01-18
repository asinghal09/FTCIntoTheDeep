package org.firstinspires.ftc.teamcode.LM5_Jan18;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

@Autonomous
@Config
public class NetAuto4Sample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmSub slidesSubsystem = new ArmSub(hardwareMap, telemetry);
        Pose2d startPos = new Pose2d(45, 63.5, Math.toRadians(270));
        drive.setPoseEstimate(startPos);


        TrajectorySequence test = drive.trajectorySequenceBuilder(startPos)
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(17))))
                .splineToSplineHeading(new Pose2d(69, 60.5, Math.toRadians(45)), Math.toRadians(45)) //to basket w preload
                .addTemporalMarker(0.05, () -> {
                    slidesSubsystem.runArmToPos(1650, 1);
                })
                .addTemporalMarker(0.25, () -> {
                    slidesSubsystem.setSlides(4100);
                })
                .resetConstraints()
                .addTemporalMarker(2.25, () -> {
                    slidesSubsystem.clawOpen();
                })
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(53,41.5,Math.toRadians(270)), Math.toRadians(270)) // to first sample
                .addTemporalMarker(3.25, () -> {
                    slidesSubsystem.setSlides(900);
                }).addTemporalMarker(3.65, () -> {
                    slidesSubsystem.runArmToPos(700,1);
                })
                .addTemporalMarker(4.5, () -> {
                    slidesSubsystem.runArmToPos(100,0.6);
                })
                .waitSeconds(1.15)
                .addTemporalMarker(5.25, () -> {
                    slidesSubsystem.clawClose();
                })
                .splineToSplineHeading(new Pose2d(68.5, 63, Math.toRadians(45)), Math.toRadians(45)) //to basket 1st sample
                .addTemporalMarker(5.75, () -> {
                    slidesSubsystem.runArmToPos(1650,1);
                })
                .addTemporalMarker(6, () -> {

                    slidesSubsystem.setSlides(4150);
                })
                .waitSeconds(0.8)
                .addTemporalMarker(8, () -> {
                    slidesSubsystem.clawOpen();
                })
                .splineToSplineHeading(new Pose2d(65.5,40.75,Math.toRadians(270)), Math.toRadians(270)) // to 2nd sample
                .addTemporalMarker(9.5, () -> {
                    slidesSubsystem.setSlides(900);
                }).addTemporalMarker(10., () -> {
                    slidesSubsystem.runArmToPos(150,0.8);
                })
                .waitSeconds(1.25)
                .addTemporalMarker(12, () -> {
                    slidesSubsystem.clawClose();
                })
                .waitSeconds(0.75)
                .splineToSplineHeading(new Pose2d(71, 64, Math.toRadians(45)), Math.toRadians(45)) //to basket 2nd sample
                .addTemporalMarker(12.5, () -> {
                    slidesSubsystem.runArmToPos(1675,1);
                })
                .addTemporalMarker(13.5, () -> {

                    slidesSubsystem.setSlides(4150);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(15.5, () -> {
                    slidesSubsystem.clawOpen();
                })
                .addTemporalMarker(17, () -> {

                    slidesSubsystem.setSlides(850);
                })
                .addTemporalMarker(17.75, () -> {

                    slidesSubsystem.runArmToPos(150,0.8);
                    slidesSubsystem.spin(.69);
                })
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(64,26, Math.toRadians(0)), Math.toRadians(270)) // to 3rd sample

                .addTemporalMarker(20, () -> {
                    slidesSubsystem.clawClose();
                })
                .addTemporalMarker(20.5, () -> {
                    slidesSubsystem.runArmToPos(1650,1);
                })

                .addTemporalMarker(21, () -> {
                    slidesSubsystem.setSlides(4200);
                    slidesSubsystem.spin(0.94);
                })
                .waitSeconds(3)
                .splineToSplineHeading(new Pose2d(71,64,Math.toRadians(45)), Math.toRadians(45))
                .addTemporalMarker(23, () -> {
                    slidesSubsystem.clawOpen();
                })
                .waitSeconds(0.25)

                .addTemporalMarker(24.5, () -> {

                    slidesSubsystem.setSlides(0);
                })
                .addTemporalMarker(25, () -> {

                    slidesSubsystem.runArmToPos(1900,0.8);
                })
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(22.5,10,Math.toRadians(0)),Math.toRadians(180)) // to level 1 ascent
                .waitSeconds(2)


                .build();



        slidesSubsystem.clawClose();
        slidesSubsystem.setJoint(1);
        slidesSubsystem.spin(0.94);


        waitForStart();
        if (!isStopRequested()){
            drive.followTrajectorySequence(test);
        }
    }

}