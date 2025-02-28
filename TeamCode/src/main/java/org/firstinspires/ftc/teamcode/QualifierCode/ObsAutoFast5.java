package org.firstinspires.ftc.teamcode.QualifierCode;
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
public class ObsAutoFast5 extends LinearOpMode {

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
                    slidesSubsystem.setJoint(0.6);
                })
                .addTemporalMarker(0.25, () -> {
                    slidesSubsystem.setSlides(1875);
                    slidesSubsystem.setJoint(0.7);
                })

                .setReversed(true)
                .addTemporalMarker(1.5,() -> {
                    slidesSubsystem.runArmToPos(700,0.7);
                    slidesSubsystem.setSlides(1850);

                })
                .addTemporalMarker(2,() -> {
                    slidesSubsystem.clawOpen();
                    slidesSubsystem.setJoint(0.6);
                })
                .waitSeconds(0.45)
                .splineToConstantHeading(new Vector2d(-14,45), Math.toRadians(90)) //back up from sub
                .splineToSplineHeading(new Pose2d(-32,35,Math.toRadians(0)),Math.toRadians(270)) // side of sub
                .splineToConstantHeading(new Vector2d(-33,14),Math.toRadians(270)) //crosses sub leg
                .splineToConstantHeading(new Vector2d(-43,12),Math.toRadians(90)) // back to line up with sample to push

                .addTemporalMarker(2.7,() -> {
                    slidesSubsystem.setSlides(0);
                })
                .addTemporalMarker(3.5,() -> {

                    slidesSubsystem.runArmToPos(400,1);
                })

                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(25)))) // slow speed for plowing 1st sample in
                .splineToConstantHeading(new Vector2d(-44,50),Math.toRadians(90)) // pushes 1st sample into obs
                /*
                .strafeRight(38)
                .back(8)
                .strafeLeft(40)
                .strafeRight(40)
                .back(8)
                .strafeLeft(40)

                 */
                .splineToConstantHeading(new Vector2d(-45,48),Math.toRadians(270)) //moves back
                .splineToSplineHeading(new Pose2d(-42,20,Math.toRadians(0)),Math.toRadians(270)) //goes forward ish to halfway to 2nd
                .splineToConstantHeading(new Vector2d(-50,12),Math.toRadians(90)) //line up with 2nd
                .splineToConstantHeading(new Vector2d(-52,50),Math.toRadians(90)) //pushes 2nd in
                .splineToConstantHeading(new Vector2d(-53,48),Math.toRadians(270)) //Moves back
                .splineToSplineHeading(new Pose2d(-50,20,Math.toRadians(0)),Math.toRadians(270)) //goes forward ish to halfway to 3rd
                .splineToConstantHeading(new Vector2d(-57,13),Math.toRadians(90)) //line up with 3rd
                .splineToConstantHeading(new Vector2d(-58,50),Math.toRadians(90)) //pushes 3rd in

                /*
                .splineToConstantHeading(new Vector2d(-45,48),Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-46,20),Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-48,12),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-52,50),Math.toRadians(90)) //pushes 2nd in
                .splineToConstantHeading(new Vector2d(-53,48),Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-54,20),Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-57,13),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-58,50),Math.toRadians(90)) //pushes 3rd in

                */


                .build();



        //init
        slidesSubsystem.clawClose();
        slidesSubsystem.setJoint(1);
        slidesSubsystem.runArmToPos(300,1);
        slidesSubsystem.spin(0.61);




        waitForStart();
        if (!isStopRequested()){
            drive.followTrajectorySequence(test);
        }
    }
}