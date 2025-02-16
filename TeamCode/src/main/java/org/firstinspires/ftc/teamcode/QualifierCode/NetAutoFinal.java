package org.firstinspires.ftc.teamcode.QualifierCode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner05x.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner05x.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

@Autonomous
@Config
public class NetAutoFinal extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmSub slidesSubsystem = new ArmSub(hardwareMap, telemetry);
        Pose2d startPos = new Pose2d(45, 63.5, Math.toRadians(270));
        drive.setPoseEstimate(startPos);


        TrajectorySequence test = drive.trajectorySequenceBuilder(startPos)
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(17))))
                .splineToSplineHeading(new Pose2d(69.5, 61, Math.toRadians(45)), Math.toRadians(45)) //to basket w preload
                .addTemporalMarker(0.05, () -> {
                    slidesSubsystem.runArmToPos(1650, 1);
                })
                .addTemporalMarker(0.25, () -> {
                    slidesSubsystem.setSlides(4300);
                })
                .resetConstraints()
                .addTemporalMarker(2.25, () -> {
                    slidesSubsystem.clawOpen();
                })
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(52.5,41.5,Math.toRadians(270)), Math.toRadians(270)) // to first sample
                .addTemporalMarker(3.25, () -> {
                    slidesSubsystem.setSlides(1100);
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
                .splineToSplineHeading(new Pose2d(68, 62.5, Math.toRadians(45)), Math.toRadians(45)) //to basket 1st sample
                .addTemporalMarker(5.75, () -> {
                    slidesSubsystem.runArmToPos(1650,1);
                })
                .addTemporalMarker(5.9, () -> {

                    slidesSubsystem.setSlides(4300);
                })
                .waitSeconds(0.8)
                .addTemporalMarker(8, () -> {
                    slidesSubsystem.clawOpen();
                })
                .splineToSplineHeading(new Pose2d(65.5,40.75,Math.toRadians(270)), Math.toRadians(270)) // to 2nd sample
                .addTemporalMarker(9.5, () -> {
                    slidesSubsystem.setSlides(1100);
                }).addTemporalMarker(10, () -> {
                    slidesSubsystem.runArmToPos(175,0.75);
                })
                .waitSeconds(1.25)
                .addTemporalMarker(12, () -> {
                    slidesSubsystem.clawClose();
                })
                .waitSeconds(0.75)
                .splineToSplineHeading(new Pose2d(70, 63, Math.toRadians(45)), Math.toRadians(45)) //to basket 2nd sample
                .addTemporalMarker(12.5, () -> {
                    slidesSubsystem.runArmToPos(1675,1);
                })
                .addTemporalMarker(13.5, () -> {

                    slidesSubsystem.setSlides(4300);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(15.5, () -> {
                    slidesSubsystem.clawOpen();
                })
                .addTemporalMarker(17, () -> {

                    slidesSubsystem.setSlides(1000);
                })
                .addTemporalMarker(17.75, () -> {

                    slidesSubsystem.runArmToPos(150,0.8);
                    slidesSubsystem.spin(.67);
                })
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(64,25, Math.toRadians(0)), Math.toRadians(270)) // to 3rd sample

                .addTemporalMarker(20, () -> {
                    slidesSubsystem.clawClose();
                })
                .addTemporalMarker(20.5, () -> {
                    slidesSubsystem.runArmToPos(1650,1);
                })

                .addTemporalMarker(21, () -> {
                    slidesSubsystem.setSlides(4300);
                    slidesSubsystem.spin(0.94);
                })
                .waitSeconds(3)
                .splineToSplineHeading(new Pose2d(71.75,64.75,Math.toRadians(45)), Math.toRadians(45))
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
                .splineToSplineHeading(new Pose2d(22.5,15,Math.toRadians(0)),Math.toRadians(180)) // to level 1 ascent
                .waitSeconds(2)


                .build();



        slidesSubsystem.clawClose();
        slidesSubsystem.setJoint(0.5);
        slidesSubsystem.spin(0.61);


        waitForStart();
        if (!isStopRequested()){
            drive.followTrajectorySequence(test);
        }
    }

}