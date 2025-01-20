package org.firstinspires.ftc.teamcode.LM5_Jan18;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.LM4_Jan5th.ArmSubOld;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;


@Autonomous
@Config
public class ObsAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmSub slidesSubsystem = new ArmSub(hardwareMap, telemetry);
        Pose2d startPos = new Pose2d(-10, 63.5, Math.toRadians(270));
        drive.setPoseEstimate(startPos);


        TrajectorySequence test = drive.trajectorySequenceBuilder(startPos)
                .splineToConstantHeading(new Vector2d(-11, 42.5), Math.toRadians(270)) // to sub with first specimen
                .addTemporalMarker(0.1, () -> {
                    slidesSubsystem.runArmToPos(1200,1);
                    slidesSubsystem.setJoint(0.75);
                })
                .addTemporalMarker(0.25, () -> {
                    slidesSubsystem.setSlides(2400);
                })

                .setReversed(true)
                .addTemporalMarker(1.75,() -> {
                    slidesSubsystem.runArmToPos(800,0.75);
                    slidesSubsystem.setSlides(2350);
                })
                .addTemporalMarker(2.6,() -> {
                    slidesSubsystem.clawOpen();
                    slidesSubsystem.setJoint(0.9);
                })
                .waitSeconds(0.7)
                .splineToConstantHeading(new Vector2d(-14,45), Math.toRadians(90)) //back up from sub
                .splineToSplineHeading(new Pose2d(-32,35,Math.toRadians(0)),Math.toRadians(270)) // side of sub
                .splineToConstantHeading(new Vector2d(-33,12),Math.toRadians(270)) //crosses sub leg
                .splineToConstantHeading(new Vector2d(-43,7),Math.toRadians(90)) // back to line up with sample to push

                .addTemporalMarker(2.7,() -> {
                    slidesSubsystem.setSlides(0);
                })
                .addTemporalMarker(3.5,() -> {

                    slidesSubsystem.runArmToPos(400,1);
                })


                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(35)))) // slow speed for plowing 1st sample in
                .splineToConstantHeading(new Vector2d(-44,60),Math.toRadians(90)) // pushes 1st sample into obs
                .resetConstraints()
                .splineToConstantHeading (new Vector2d(-35,20),Math.toRadians(270)) //goes back near sub for 2nd sample
                .splineToConstantHeading (new Vector2d(-50,5),Math.toRadians(90)) //lines up with 2nd sample
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(35)))) //slow speed for plowing 2nd one in
                .splineToConstantHeading (new Vector2d(-51,60),Math.toRadians(90)) //pushes 2nd sample in
                .resetConstraints()
                .addTemporalMarker(11,() -> {
                    slidesSubsystem.setJoint(1);
                    slidesSubsystem.runArmToPos(0,1);
                    slidesSubsystem.setSlides(0);

        }       )
                .waitSeconds(5)

                /*
                .splineToSplineHeading(new Pose2d(-50,40,Math.toRadians(90)),Math.toRadians(0))
                .waitSeconds(0.5)
                .forward(10)
                .waitSeconds(2)

                .addTemporalMarker(11,() -> {
                    slidesSubsystem.setJoint(0);
                })
                .addTemporalMarker(11,() -> {
                    slidesSubsystem.setSlides(1000);
                })
                .addTemporalMarker(13.5,() -> {
                    slidesSubsystem.clawClose();
                })
                .addTemporalMarker(14,() -> {

                    slidesSubsystem.runArmToPos(1300,1);
                    slidesSubsystem.setSlides(675);
                })
                .splineToSplineHeading(new Pose2d(7, 32, Math.toRadians(270)), Math.toRadians(270)) //to sub with                .waitSeconds(1)
                .addTemporalMarker(18,() -> {

                    slidesSubsystem.runArmToPos(950,0.75);
                })
                .waitSeconds(5)
                /*.back(20)
                .addTemporalMarker(25,() -> {

                    slidesSubsystem.clawOpen();
                })
                .waitSeconds(1)
                .addTemporalMarker(26,() -> {

                    slidesSubsystem.runArmToPos(0,1);
                    slidesSubsystem.setSlides(0);
                })
                .splineToConstantHeading(new Vector2d(-50,40),Math.toRadians(180))

                 */
                .build();

        //init
        slidesSubsystem.clawClose();
        slidesSubsystem.setJoint(1);
        slidesSubsystem.runArmToPos(300,1);
        slidesSubsystem.spin(0.94);




        waitForStart();
        if (!isStopRequested()){
            drive.followTrajectorySequence(test);
        }
    }
}