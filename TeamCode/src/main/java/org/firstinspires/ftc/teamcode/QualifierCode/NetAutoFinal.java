package org.firstinspires.ftc.teamcode.QualifierCode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
        Pose2d startPos = new Pose2d(36, 63.5, Math.toRadians(180));
        drive.setPoseEstimate(startPos);


        TrajectorySequence test = drive.trajectorySequenceBuilder(startPos)
                .setReversed(true)
                .waitSeconds(0.3)
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(10))))
                .splineToSplineHeading(new Pose2d(50, 50, Math.toRadians(225)), Math.toRadians(0)) //to basket w preload
                .addTemporalMarker(0.05, () -> {
                    slidesSubsystem.runArmToPos(1725, 1);
                })
                .addTemporalMarker(0.2, () -> {
                    slidesSubsystem.setSlides(4050);
                    slidesSubsystem.setJoint(0);
                })
                .resetConstraints()
                .addTemporalMarker(2.8, () -> {
                    slidesSubsystem.clawOpen();
                })
                .setReversed(false)
                .waitSeconds(0.3)
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(15))))
                .splineToSplineHeading(new Pose2d(46.75,41.5,Math.toRadians(270)), Math.toRadians(270)) // to first sample
                .addTemporalMarker(3.25, () -> {
                    slidesSubsystem.runArmToPos(1650,1);
                })
                .addTemporalMarker(3.5, () -> {
                    slidesSubsystem.setSlides(1200);
                })
                .addTemporalMarker(4.2, () -> {
                    slidesSubsystem.runArmToPos(700,0.8);
                    slidesSubsystem.setJoint(1);
                })
                .addTemporalMarker(4.5, () -> {
                    slidesSubsystem.runArmToPos(115,0.6);

                })
                .waitSeconds(1.35)
                .addTemporalMarker(5.25, () -> {
                    slidesSubsystem.clawClose();
                })
                .setReversed(true)
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(12))))
                .splineToSplineHeading(new Pose2d(50, 50, Math.toRadians(225)), Math.toRadians(45)) //to basket 1st sample
                .addTemporalMarker(5.5, () -> {
                    slidesSubsystem.runArmToPos(1550,0.8);
                    slidesSubsystem.setJoint(0.1);
                })
                .addTemporalMarker(5.75, () -> {

                    slidesSubsystem.setSlides(4100);
                })
                .addTemporalMarker(6.25, () -> {
                    slidesSubsystem.runArmToPos(1725,0.3);
                    slidesSubsystem.setJoint(0.1);
                })
                .waitSeconds(1)
                .addTemporalMarker(7.1, () -> {
                    slidesSubsystem.clawOpen();
                })

                .setReversed(false)
                .splineToSplineHeading(new Pose2d(55,41.5,Math.toRadians(270)), Math.toRadians(0)) // to 2nd sample
                .addTemporalMarker(7.5, () -> {
                    slidesSubsystem.runArmToPos(1650, 1);
                })
                .addTemporalMarker(7.75, () -> {
                    slidesSubsystem.setSlides(1150);
                    slidesSubsystem.setJoint(1);
                })
                .addTemporalMarker(8.25, () -> {
                    slidesSubsystem.runArmToPos(105,0.35);

                })
                .waitSeconds(1.25)
                .addTemporalMarker(10, () -> {
                    slidesSubsystem.clawClose();
                })

                .waitSeconds(0.75)

                .setReversed(true)

                .splineToConstantHeading(new Vector2d(46.75,41.5), Math.toRadians(270)) // to first sample pos
                .splineToSplineHeading(new Pose2d(50, 50, Math.toRadians(225)), Math.toRadians(90)) //to basket 2nd sample
                .addTemporalMarker(10.5, () -> {
                    slidesSubsystem.runArmToPos(1715,0.7);
                    slidesSubsystem.setJoint(0.1);
                    slidesSubsystem.setSlides(4100);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(13, () -> {
                    slidesSubsystem.clawOpen();
                })
                .addTemporalMarker(13.5, () -> {

                    slidesSubsystem.runArmToPos(1500,1);
                })
                .addTemporalMarker(14, () -> {

                    slidesSubsystem.setSlides(1425);
                    slidesSubsystem.setJoint(1);
                    slidesSubsystem.spin(.5);
                })
                .addTemporalMarker(16, () -> {

                    slidesSubsystem.runArmToPos(300,0.7);

                })
                .addTemporalMarker(17.25, () -> {

                    slidesSubsystem.runArmToPos(100,0.3);

                })
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(48,28, Math.toRadians(0)), Math.toRadians(270)) // to 3rd sample
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(10))))
                .splineToConstantHeading(new Vector2d(52,25.25),Math.toRadians(0))

                .addTemporalMarker(18, () -> {
                    slidesSubsystem.clawClose();
                })

                .addTemporalMarker(18.5, () -> {
                    slidesSubsystem.runArmToPos(1725,0.7);
                })

                .addTemporalMarker(19, () -> {
                    slidesSubsystem.setSlides(4100);
                    slidesSubsystem.setJoint(0.1);
                    slidesSubsystem.spin(0.75);
                })
                .waitSeconds(1.5)
                .setReversed(true)
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(13))))
                .splineToSplineHeading(new Pose2d(50,50,Math.toRadians(225)), Math.toRadians(45)) // to basket w 3rd
                .addTemporalMarker(21, () -> {
                    slidesSubsystem.clawOpen();
                })
                .waitSeconds(0.25)

                .addTemporalMarker(21.5, () -> {

                    slidesSubsystem.runArmToPos(1400,1);
                    slidesSubsystem.setJoint(1);
                })
                .addTemporalMarker(22.5, () -> {

                    slidesSubsystem.setSlides(0);
                })
                .addTemporalMarker(25.5, () -> {

                    slidesSubsystem.runArmToPos(1900,0.7);
                })
                .setReversed(false)
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(17))))

                .splineToSplineHeading(new Pose2d(27,13,Math.toRadians(0)),Math.toRadians(180)) // to level 1 ascent
                .waitSeconds(2)

                .build();



        slidesSubsystem.clawClose();
        slidesSubsystem.setJoint(1);
        slidesSubsystem.spin(0.75);
        slidesSubsystem.runArmToPos(750,0.7);


        waitForStart();
        if (!isStopRequested()){
            drive.followTrajectorySequence(test);
        }
    }

}