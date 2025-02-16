package org.firstinspires.ftc.teamcode.LM4_Jan5th;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner05x.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner05x.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

@Disabled
@Autonomous
@Config
public class ObsAutoFinal extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmSubOld slidesSubsystem = new ArmSubOld(hardwareMap, telemetry);
        Pose2d startPos = new Pose2d(-10, 63.5, Math.toRadians(270));
        drive.setPoseEstimate(startPos);


        TrajectorySequence test = drive.trajectorySequenceBuilder(startPos)
                .splineToConstantHeading(new Vector2d(-11, 35), Math.toRadians(270)) // to sub with first specimen
                .addTemporalMarker(0.1, () -> {
                    slidesSubsystem.runArmToPos(900,1);

                })
                .addTemporalMarker(0.25, () -> {
                    slidesSubsystem.setSlides(2570);
                })

                .setReversed(true)
                .addTemporalMarker(1.75,() -> {
                    slidesSubsystem.runArmToPos(1100,0.5);
                })
                .addTemporalMarker(2.9,() -> {
                    slidesSubsystem.clawOpen();
                })
                .waitSeconds(0.9)
                .splineToConstantHeading(new Vector2d(-14,45), Math.toRadians(90)) //back up from sub
                .splineToSplineHeading(new Pose2d(-46,35,Math.toRadians(0)),Math.toRadians(270)) // side of sub
                .splineToConstantHeading(new Vector2d(-47,0),Math.toRadians(270)) //crosses sub leg
                .splineToConstantHeading(new Vector2d(-59,-17),Math.toRadians(90)) // back to line up with sample to push

                .addTemporalMarker(3.25,() -> {
                    slidesSubsystem.setSlides(0);
                })
                .addTemporalMarker(3.5,() -> {

                    slidesSubsystem.runArmToPos(100,1);
                })


                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(35)))) // slow speed for plowing 1st sample in
                .splineToConstantHeading(new Vector2d(-61,55),Math.toRadians(90)) // pushes 1st sample into obs
                .resetConstraints()
                .splineToConstantHeading (new Vector2d(-55,30),Math.toRadians(270)) //goes back near sub for 2nd sample
                .splineToConstantHeading (new Vector2d(-68,-15),Math.toRadians(90)) //lines up with 2nd sample
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(35)))) //slow speed for plowing 2nd one in
                .splineToConstantHeading (new Vector2d(-69,55),Math.toRadians(90)) //pushes 2nd sample in
                .resetConstraints()
                .splineToSplineHeading(new Pose2d(-57,30,Math.toRadians(90)),Math.toRadians(0))
                .waitSeconds(0.5)
                .forward(17)
                .waitSeconds(2)
                .addTemporalMarker(17,() -> {
                    slidesSubsystem.clawClose();
                })
                .addTemporalMarker(19.5,() -> {

                    slidesSubsystem.runArmToPos(900,1);
                    slidesSubsystem.setSlides(2570);
                })
                .splineToSplineHeading(new Pose2d(7, 36.75, Math.toRadians(270)), Math.toRadians(270)) //to sub with 2nd specimen
                .waitSeconds(1)
                .addTemporalMarker(23,() -> {

                    slidesSubsystem.runArmToPos(1100,0.5);
                })
                .back(20)
                .addTemporalMarker(25,() -> {

                    slidesSubsystem.clawOpen();
                })
                .waitSeconds(1)
                .addTemporalMarker(26,() -> {

                    slidesSubsystem.runArmToPos(0,1);
                    slidesSubsystem.setSlides(0);
                })
                .strafeRight(60)
                .build();

        slidesSubsystem.clawClose();

        waitForStart();
        if (!isStopRequested()){
            drive.followTrajectorySequence(test);
        }
    }
}