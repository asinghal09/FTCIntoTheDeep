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
public class ObsAuto5specimen extends LinearOpMode {

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

                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(35)))) // slow speed for plowing 1st sample in
                .splineToConstantHeading(new Vector2d(-44,50),Math.toRadians(90)) // pushes 1st sample into obs

                .resetConstraints()
                .splineToConstantHeading(new Vector2d(-45,48),Math.toRadians(270)) //moves back
                .splineToSplineHeading(new Pose2d(-40,32,Math.toRadians(0)),Math.toRadians(270)) //goes forward ish to halfway to 2nd
                .splineToConstantHeading(new Vector2d(-33,14),Math.toRadians(270)) //line up with 2nd
                .splineToConstantHeading(new Vector2d(-40,12), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-41,55),Math.toRadians(115)) //pushes 2nd in
                .splineToConstantHeading(new Vector2d(-42,48),Math.toRadians(270)) //moves back
                .splineToSplineHeading(new Pose2d(-38,32,Math.toRadians(0)),Math.toRadians(270)) //goes forward ish to halfway to 2nd
                .splineToConstantHeading(new Vector2d(-33,14),Math.toRadians(270)) //line up with 3nd
                .splineToConstantHeading(new Vector2d(-48,12), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-53,53),Math.toRadians(115)) //pushes 3nd in


                .splineToSplineHeading(new Pose2d(-45,55,Math.toRadians(90)),Math.toRadians(0)) // lines up with specimen in obs
                .addTemporalMarker(10,() -> {
                    slidesSubsystem.clawOpen();
                    slidesSubsystem.setJoint(0.6);
                    slidesSubsystem.runArmToPos(390,1);
                    slidesSubsystem.setSlides(200);
                    slidesSubsystem.spin(0.75);
                }       )
                .waitSeconds(2)


                .addTemporalMarker(13,() -> {
                    slidesSubsystem.setSlides(500);
                })
                .addTemporalMarker(13.5,() -> {
                    slidesSubsystem.clawClose();
                })
                .addTemporalMarker(14,() -> {
                    slidesSubsystem.runArmToPos(2050,1);
                    slidesSubsystem.setSlides(1400);
                    slidesSubsystem.setJoint(0);
                    slidesSubsystem.spin(0.05);
                })
                .resetConstraints()
                .splineToConstantHeading(new Vector2d(-20,50),Math.toRadians(0)) // to sub
                .splineToConstantHeading(new Vector2d(-5,47),Math.toRadians(0))  // to sub still
                .waitSeconds(1.75)
                .addTemporalMarker(17.75,() -> {
                    slidesSubsystem.runArmToPos(2400,0.75);
                    slidesSubsystem.setSlides(300);

                })
                .addTemporalMarker(18.25,() -> {
                    slidesSubsystem.clawOpen();

                })
                .addTemporalMarker(18.75,() -> {

                    slidesSubsystem.setJoint(0.6);
                    slidesSubsystem.runArmToPos(390,1);
                    slidesSubsystem.setSlides(500);
                    slidesSubsystem.spin(0.75);

                })
                .addTemporalMarker(21.75,() -> {
                    slidesSubsystem.setSlides(400);
                })
                .addTemporalMarker(22.25,() -> {
                    slidesSubsystem.clawClose();
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-50,55),Math.toRadians(90)) // lines up with specimen in obs
                .waitSeconds(2.5)
                .addTemporalMarker(22.75,() -> {
                    slidesSubsystem.runArmToPos(2050,1);
                    slidesSubsystem.setSlides(1400);
                    slidesSubsystem.setJoint(0);
                    slidesSubsystem.spin(0.05);
                })
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-20,50),Math.toRadians(0)) // to sub
                .splineToConstantHeading(new Vector2d(0,47),Math.toRadians(0))  // to sub still
                .waitSeconds(1.75)
                .addTemporalMarker(26.75,() -> {
                    slidesSubsystem.runArmToPos(2400,0.75);
                    slidesSubsystem.setSlides(300);

                })
                .addTemporalMarker(27.25,() -> {
                    slidesSubsystem.clawOpen();

                })

                //.splineToConstantHeading(new Vector2d(-53,48),Math.toRadians(270)) //Moves back
                //.splineToSplineHeading(new Pose2d(-50,20,Math.toRadians(0)),Math.toRadians(270)) //goes forward ish to halfway to 3rd
                //.splineToConstantHeading(new Vector2d(-57,13),Math.toRadians(90)) //line up with 3rd
                //.splineToConstantHeading(new Vector2d(-58,50),Math.toRadians(90)) //pushes 3rd in

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
        slidesSubsystem.spin(0.68);




        waitForStart();
        if (!isStopRequested()){
            drive.followTrajectorySequence(test);
        }
    }
}