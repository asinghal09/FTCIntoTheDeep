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
public class ObsAutoNew extends LinearOpMode {

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
                //.setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(50))))
                .splineToLinearHeading(new Pose2d(-69,25,Math.toRadians(90)), Math.toRadians(180)) // to obs
                .splineToConstantHeading(new Vector2d(-69.01,34),Math.toRadians(90)) // lines up with specimen
                .splineToLinearHeading(new Pose2d(-6,25,Math.toRadians(270)), Math.toRadians(180)) // to sub with specimen
                /*.splineToSplineHeading(new Pose2d(-25,58, Math.toRadians(180)), Math.toRadians(180)) //to obs zone, lines up with specimen
                .splineToConstantHeading(new Vector2d(-48,59.75),Math.toRadians(180)) //drives into specimen

                 */
                //.resetConstraints()
                .waitSeconds(0.5)

                .addTemporalMarker(3.5,() -> {
                    slidesSubsystem.setSlides(815);                        //picking up pos
                    slidesSubsystem.runArmToPos(150,0.8);
                })
                .addTemporalMarker(7.15,()-> {
                    slidesSubsystem.clawClose();
                })
                .addTemporalMarker(7.5,()-> {
                    slidesSubsystem.runArmToPos(900,1);
                })
                .addTemporalMarker(8,()->{
                    slidesSubsystem.setSlides(1700);
                })
                //.splineToSplineHeading(new Pose2d(-3, 37.75, Math.toRadians(270)), Math.toRadians(270)) //to sub with 2nd specimen
                .addTemporalMarker(11.5,()->{
                    slidesSubsystem.setSlides(2570);
                })
                .addTemporalMarker(12,() -> {
                    slidesSubsystem.runArmToPos(1100,0.5);
                })
                .addTemporalMarker(13.25,() -> {
                    slidesSubsystem.clawOpen();
                })
                .waitSeconds(1.25)
                .splineToConstantHeading(new Vector2d(-3.5,45), Math.toRadians(90)) //back up from sub
                .splineToSplineHeading(new Pose2d(-43,35,Math.toRadians(0)),Math.toRadians(270)) // side of sub
                .splineToConstantHeading(new Vector2d(-45,0),Math.toRadians(270)) //crosses sub leg
                .splineToConstantHeading(new Vector2d(-50,-17),Math.toRadians(90)) // back to line up with sample to push

                .addTemporalMarker(14,() -> {
                    slidesSubsystem.setSlides(0);
                })
                .addTemporalMarker(14.25,() -> {

                    slidesSubsystem.runArmToPos(100,1);
                })


                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(35)))) // slow speed for plowing 1st sample in
                .splineToConstantHeading(new Vector2d(-51,55),Math.toRadians(90)) // pushes 1st sample into obs
                .resetConstraints()
                .splineToConstantHeading (new Vector2d(-55,30),Math.toRadians(270)) //goes back near sub for 2nd sample
                .splineToConstantHeading (new Vector2d(-70,-15),Math.toRadians(90)) //lines up with 2nd sample
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(35)))) //slow speed for plowing 2nd one in
                .splineToConstantHeading (new Vector2d(-71,55),Math.toRadians(90)) //pushes 2nd sample in
                .resetConstraints()
                .splineToSplineHeading(new Pose2d(-60,40,Math.toRadians(90)),Math.toRadians(0))
                .build();

        slidesSubsystem.clawClose();

        waitForStart();
        if (!isStopRequested()){
            drive.followTrajectorySequence(test);
        }
    }
}