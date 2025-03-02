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
public class ObsAutoBigLoop4 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmSub slidesSubsystem = new ArmSub(hardwareMap, telemetry);
        Pose2d startPos = new Pose2d(-10, 63.5, Math.toRadians(270));
        drive.setPoseEstimate(startPos);


        TrajectorySequence test = drive.trajectorySequenceBuilder(startPos)
                .splineToConstantHeading(new Vector2d(-10.5, 38), Math.toRadians(270)) // to sub with first specimen
                .addTemporalMarker(0.1, () -> {
                    slidesSubsystem.runArmToPos(1420,1);

                })
                .addTemporalMarker(0.25, () -> {
                    slidesSubsystem.setSlides(1130);
                    slidesSubsystem.setJoint(1);
                })

                .setReversed(true)
                .addTemporalMarker(1.5,() -> {

                    slidesSubsystem.setSlides(200);

                })
                .addTemporalMarker(2,() -> {
                    slidesSubsystem.clawOpen();

                })
                .waitSeconds(0.45)
                .splineToConstantHeading(new Vector2d(-14,45), Math.toRadians(90)) //back up from sub
                .splineToSplineHeading(new Pose2d(-32,35,Math.toRadians(0)),Math.toRadians(270)) // side of sub
                .splineToConstantHeading(new Vector2d(-33,14),Math.toRadians(270)) //crosses sub leg
                .splineToConstantHeading(new Vector2d(-41,12),Math.toRadians(90)) // back to line up with sample to push

                .addTemporalMarker(2.7,() -> {
                    slidesSubsystem.setSlides(0);
                })
                .addTemporalMarker(3.5,() -> {

                    slidesSubsystem.runArmToPos(400,1);
                })

                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(35)))) // slow speed for plowing 1st sample in
                .splineToConstantHeading(new Vector2d(-42,53),Math.toRadians(90)) // pushes 1st sample into obs

                .resetConstraints()
                .splineToConstantHeading(new Vector2d(-45,48),Math.toRadians(270)) //moves back
                .splineToSplineHeading(new Pose2d(-40,32,Math.toRadians(0)),Math.toRadians(270)) //goes forward ish to halfway to 2nd
                .splineToConstantHeading(new Vector2d(-33,14),Math.toRadians(270)) //line up with 2nd
                .splineToConstantHeading(new Vector2d(-42,12), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-43,53),Math.toRadians(115)) //pushes 2nd in

                /*.splineToConstantHeading(new Vector2d(-42,48),Math.toRadians(270)) //moves back
                .splineToSplineHeading(new Pose2d(-38,32,Math.toRadians(0)),Math.toRadians(270)) //goes forward ish to halfway to 2nd
                .splineToConstantHeading(new Vector2d(-33,14),Math.toRadians(270)) //line up with 3nd
                .splineToConstantHeading(new Vector2d(-48,12), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-53,53),Math.toRadians(115)) //pushes 3nd in

                 */


                .splineToSplineHeading(new Pose2d(-45,55,Math.toRadians(90)),Math.toRadians(270)) // lines up with 1st specimen in obs
                .addTemporalMarker(9,() -> {
                    slidesSubsystem.clawOpen();
                    slidesSubsystem.setJoint(0.5);
                    slidesSubsystem.runArmToPos(450,1);
                    slidesSubsystem.setSlides(200);
                    slidesSubsystem.spin(0.75);
                })
                .waitSeconds(1.25)


                .addTemporalMarker(10.4,() -> {
                    slidesSubsystem.setSlides(650);
                })
                .addTemporalMarker(10.9,() -> {
                    slidesSubsystem.clawClose();
                })
                .addTemporalMarker(11.15,() -> {
                    slidesSubsystem.runArmToPos(2050,0.5);
                    slidesSubsystem.setJoint(0);
                    slidesSubsystem.spin(0.05);
                })
                .addTemporalMarker(11.35,() -> {
                    slidesSubsystem.setSlides(1400);
                })
                .resetConstraints()
                .splineToConstantHeading(new Vector2d(-15,60),Math.toRadians(0)) // to sub with 1st
                .splineToConstantHeading(new Vector2d(-5,48.25),Math.toRadians(0))  // to sub still
                //.waitSeconds(1)
                .addTemporalMarker(13.25,() -> {
                    slidesSubsystem.runArmToPos(2400,0.75);
                    slidesSubsystem.setSlides(325);

                })
                .addTemporalMarker(13.75,() -> {
                    slidesSubsystem.clawOpen();

                })
                .addTemporalMarker(14.5,() -> {

                    slidesSubsystem.setJoint(0.5);
                    slidesSubsystem.runArmToPos(475,1);
                    slidesSubsystem.setSlides(200);
                    slidesSubsystem.spin(0.75);

                })
                .addTemporalMarker(16.25,() -> {
                    slidesSubsystem.setSlides(575);
                })
                .addTemporalMarker(16.75,() -> {
                    slidesSubsystem.clawClose();
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-45,56),Math.toRadians(90)) // lines up with 2nd specimen in obs
                .waitSeconds(2)
                .addTemporalMarker(17.25,() -> {
                    slidesSubsystem.runArmToPos(2050,0.5);
                    slidesSubsystem.setJoint(0);
                    slidesSubsystem.spin(0.05);
                })
                .addTemporalMarker(17.5,() -> {
                    slidesSubsystem.setSlides(1400);
                })
                .setReversed(true)

                .splineToConstantHeading(new Vector2d(-45, 55),Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-20,60),Math.toRadians(0)) // to sub with 2nd one
                .splineToConstantHeading(new Vector2d(-10,65), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(0,47.25),Math.toRadians(270))  // to sub still
                .waitSeconds(0.5)
                .addTemporalMarker(20.25,() -> {
                    slidesSubsystem.runArmToPos(2400,0.75);
                    slidesSubsystem.setSlides(325);

                })
                .addTemporalMarker(20.75,() -> {
                    slidesSubsystem.clawOpen();

                })

                .addTemporalMarker(21.25,() -> {

                    slidesSubsystem.setJoint(0.5);
                    slidesSubsystem.runArmToPos(475,1);
                    slidesSubsystem.setSlides(200);
                    slidesSubsystem.spin(0.75);

                })
                .addTemporalMarker(23.25,() -> {
                    slidesSubsystem.setSlides(575);
                })
                .addTemporalMarker(23.75,() -> {
                    slidesSubsystem.clawClose();
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-35,50),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-45,56),Math.toRadians(90)) // lines up with 3nd specimen in obs
                .waitSeconds(1.75)
                .addTemporalMarker(24,() -> {
                    slidesSubsystem.runArmToPos(2050,0.5);
                    slidesSubsystem.setJoint(0);
                    slidesSubsystem.spin(0.05);
                })
                .addTemporalMarker(24.25,() -> {
                    slidesSubsystem.setSlides(1400);
                    })
                .setReversed(true)

                .splineToConstantHeading(new Vector2d(-45, 55),Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-20,60),Math.toRadians(0)) // to sub with 3nd one
                .splineToConstantHeading(new Vector2d(-10,65), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(5,47.25),Math.toRadians(270))  // to sub still
                .waitSeconds(0.5)
                .addTemporalMarker(27.5,() -> {
                    slidesSubsystem.runArmToPos(2400,0.75);
                    slidesSubsystem.setSlides(325);

                })
                .addTemporalMarker(28,() -> {
                    slidesSubsystem.clawOpen();

                })
                .addTemporalMarker(28.25,() -> {
                    slidesSubsystem.runArmToPos(200,1);
                    slidesSubsystem.setJoint(0.5);
                    slidesSubsystem.setSlides(0);
                    slidesSubsystem.spin(0.68);

                })
                .splineToConstantHeading(new Vector2d(-35,50),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-45,52),Math.toRadians(90))

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