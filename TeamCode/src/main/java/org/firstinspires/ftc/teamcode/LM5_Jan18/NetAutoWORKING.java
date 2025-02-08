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
public class NetAutoWORKING extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmSubOld slidesSubsystem = new ArmSubOld(hardwareMap, telemetry);
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
                .addTemporalMarker(6, () -> {
                    slidesSubsystem.runArmToPos(1675,1);
                })
                .addTemporalMarker(6.25, () -> {

                    slidesSubsystem.setSlides(4150);
                })
                .waitSeconds(1.25)
                .addTemporalMarker(8.7, () -> {
                    slidesSubsystem.clawOpen();
                })
                .splineToSplineHeading(new Pose2d(66,40.75,Math.toRadians(270)), Math.toRadians(270)) // to 2nd sample
                .addTemporalMarker(9.5, () -> {
                    slidesSubsystem.setSlides(900);
                }).addTemporalMarker(10.5, () -> {
                    slidesSubsystem.runArmToPos(100,0.8);
                })
                .waitSeconds(1.25)
                .addTemporalMarker(12.5, () -> {
                    slidesSubsystem.clawClose();
                })
                .waitSeconds(0.75)
                .splineToSplineHeading(new Pose2d(71, 64, Math.toRadians(45)), Math.toRadians(45)) //to basket 2nd sample
                .addTemporalMarker(12.75, () -> {
                    slidesSubsystem.runArmToPos(1690,1);
                })
                .addTemporalMarker(13.75, () -> {

                    slidesSubsystem.setSlides(4150);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(16, () -> {
                    slidesSubsystem.clawOpen();
                })
                .addTemporalMarker(18, () -> {

                    slidesSubsystem.setSlides(0);
                })
                .addTemporalMarker(18.5, () -> {

                    slidesSubsystem.runArmToPos(1900,0.8);
                })
                .setReversed(true)
                //.setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(35))))
                //.splineToSplineHeading(new Pose2d(65,0,Math.toRadians(180)),Math.toRadians(0)) // to 3rd sample
                //.splineToConstantHeading(new Vector2d(64,60),Math.toRadians(90)) // push 3rd in
                //.resetConstraints()
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