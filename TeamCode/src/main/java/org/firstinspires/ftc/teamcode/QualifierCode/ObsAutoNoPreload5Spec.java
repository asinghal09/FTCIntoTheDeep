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
public class ObsAutoNoPreload5Spec extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmSub slidesSubsystem = new ArmSub(hardwareMap, telemetry);
        Pose2d startPos = new Pose2d(-21, 63.5, Math.toRadians(0));
        drive.setPoseEstimate(startPos);


        TrajectorySequence test = drive.trajectorySequenceBuilder(startPos)
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(41))))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-26,35),Math.toRadians(270)) // side of sub
                .splineToConstantHeading(new Vector2d(-27,14),Math.toRadians(270)) //crosses sub leg
                .splineToConstantHeading(new Vector2d(-36,12),Math.toRadians(180)) // back to line up with sample to push

                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(30)))) // slow speed for plowing 1st sample in
                .splineToConstantHeading(new Vector2d(-38,53),Math.toRadians(90)) // pushes 1st sample into obs

                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(41))))

                .splineToConstantHeading(new Vector2d(-43,48),Math.toRadians(180)) //moves back
                .splineToConstantHeading(new Vector2d(-37,32),Math.toRadians(270)) //goes forward ish to halfway to 2nd
                .splineToConstantHeading(new Vector2d(-33,14),Math.toRadians(270)) //line up with 2nd
                .splineToConstantHeading(new Vector2d(-44,12), Math.toRadians(180))

                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(35)))) // slow speed for plowing 2nd sample in
                .splineToConstantHeading(new Vector2d(-46,54),Math.toRadians(115)) //pushes 2nd in

                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(41))))

                .splineToConstantHeading(new Vector2d(-47,48),Math.toRadians(180)) //moves back
                .splineToConstantHeading(new Vector2d(-40,32),Math.toRadians(270)) //goes forward ish to halfway to 2nd
                .splineToConstantHeading(new Vector2d(-45,14),Math.toRadians(270)) //line up with 3nd
                .splineToConstantHeading(new Vector2d(-51,12), Math.toRadians(180))

                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(35)))) // slow speed for plowing 3rd sample in
                .splineToConstantHeading(new Vector2d(-51.5,54),Math.toRadians(90)) //pushes 3nd in


                .splineToSplineHeading(new Pose2d(-45,50,Math.toRadians(90)),Math.toRadians(270)) // lines up with 1st specimen in obs
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(10)))) // slow speed
                .splineToConstantHeading(new Vector2d(-45,59),Math.toRadians(90)) // lines up with 1st specimen in obs
                .resetConstraints()

                .addTemporalMarker(5,() -> {
                    slidesSubsystem.clawOpen();
                    slidesSubsystem.setJoint(0.4);
                    slidesSubsystem.runArmToPos(500,1);
                    slidesSubsystem.setSlides(0);
                    slidesSubsystem.spin(0.75);
                })
                .waitSeconds(0.2)

                .addTemporalMarker(10.5,() -> {
                    slidesSubsystem.clawClose();
                })

                .addTemporalMarker( 10.8,() -> {
                    slidesSubsystem.runArmToPos(2050,0.35);
                    slidesSubsystem.setJoint(0);
                    slidesSubsystem.spin(0.05);
                })
                .addTemporalMarker(11,() -> {
                    slidesSubsystem.setSlides(1400);
                })
                .resetConstraints()
                .splineToConstantHeading(new Vector2d(-15,60),Math.toRadians(0)) // to sub with 1st
                .splineToConstantHeading(new Vector2d(-10,46),Math.toRadians(0))  // to sub still

                .addTemporalMarker(13.15,() -> {
                    slidesSubsystem.runArmToPos(2400,0.75);
                    slidesSubsystem.setSlides(325);

                })
                .addTemporalMarker(13.65,() -> {
                    slidesSubsystem.clawOpen();

                })

                .addTemporalMarker(13.8,() -> {

                    slidesSubsystem.setJoint(0.4);
                    slidesSubsystem.runArmToPos(500,1);
                    slidesSubsystem.setSlides(0);
                    slidesSubsystem.spin(0.75);

                })
                .addTemporalMarker(15.5,() -> {
                    slidesSubsystem.clawClose();
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-45,54),Math.toRadians(90)) // lines up with 2nd specimen in obs
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(17)))) // slow speed for plowing 1st sample in
                .splineToConstantHeading(new Vector2d(-45,58.5),Math.toRadians(90))
                .resetConstraints()


                .waitSeconds(0.4)

                .addTemporalMarker(15.8,() -> {
                    slidesSubsystem.runArmToPos(2050,0.35);
                    slidesSubsystem.setJoint(0);
                    slidesSubsystem.spin(0.05);
                })
                .addTemporalMarker(16,() -> {
                    slidesSubsystem.setSlides(1400);
                })
                .setReversed(true)

                .splineToConstantHeading(new Vector2d(-45, 55),Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-20,57),Math.toRadians(0)) // to sub with 2nd one
                .splineToConstantHeading(new Vector2d(-10,55), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-6,45),Math.toRadians(270))  // to sub still
                .addTemporalMarker(18.1,() -> {
                    slidesSubsystem.runArmToPos(2400,0.75);
                    slidesSubsystem.setSlides(325);

                })
                .addTemporalMarker(18.6,() -> {
                    slidesSubsystem.clawOpen();

                })
                .addTemporalMarker(18.9,() -> {

                    slidesSubsystem.setJoint(0.4);
                    slidesSubsystem.runArmToPos(500,1);
                    slidesSubsystem.setSlides(0);
                    slidesSubsystem.spin(0.75);

                })
                /*
                .addTemporalMarker(18,() -> {
                    slidesSubsystem.clawClose();
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-30,46),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-45,54),Math.toRadians(90)) // lines up with 3nd specimen in obs
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(17)))) // slow speed for plowing 1st sample in
                .splineToConstantHeading(new Vector2d(-45,58.5),Math.toRadians(90))
                .resetConstraints()

                .waitSeconds(0.4)

                .addTemporalMarker(18.3,() -> {
                    slidesSubsystem.runArmToPos(2050,0.35);
                    slidesSubsystem.setJoint(0);
                    slidesSubsystem.spin(0.05);
                })
                .addTemporalMarker(18.5,() -> {
                    slidesSubsystem.setSlides(1400);
                    })
                .setReversed(true)

                .splineToConstantHeading(new Vector2d(-45, 55),Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-20,56),Math.toRadians(0)) // to sub with 3nd one
                .splineToConstantHeading(new Vector2d(-10,57), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-2,44.5),Math.toRadians(270))  // to sub still
                .waitSeconds(0.2)
                .addTemporalMarker(20.5,() -> {
                    slidesSubsystem.runArmToPos(2400,0.75);
                    slidesSubsystem.setSlides(325);

                })
                .addTemporalMarker(21,() -> {
                    slidesSubsystem.clawOpen();

                })

                .addTemporalMarker(21.15,() -> {
                    slidesSubsystem.runArmToPos(500,1);
                    slidesSubsystem.setJoint(0.4);
                    slidesSubsystem.setSlides(0);
                    slidesSubsystem.spin(0.68);

                })
                .addTemporalMarker(23,() -> {
                    slidesSubsystem.clawClose();
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-35,47),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-45,53),Math.toRadians(90)) // lines up with 4th specimen in obs
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(17)))) // slow speed for plowing 1st sample in
                .splineToConstantHeading(new Vector2d(-45,58.5),Math.toRadians(90))
                .resetConstraints()
                .waitSeconds(0.2)

                .addTemporalMarker(23.3,() -> {
                    slidesSubsystem.runArmToPos(2050,0.35);
                    slidesSubsystem.setJoint(0);
                    slidesSubsystem.spin(0.05);
                })
                .addTemporalMarker(23.5,() -> {
                    slidesSubsystem.setSlides(1400);
                })
                .setReversed(true)

                .splineToConstantHeading(new Vector2d(-45, 55),Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-20,56),Math.toRadians(0)) // to sub with 4nd one
                .splineToConstantHeading(new Vector2d(-10,57), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(2,44),Math.toRadians(270))  // to sub still

                .addTemporalMarker(25.25,() -> {
                    slidesSubsystem.runArmToPos(2400,0.75);
                    slidesSubsystem.setSlides(325);

                })
                .addTemporalMarker(26,() -> {
                    slidesSubsystem.clawOpen();

                })

                .addTemporalMarker(26.15,() -> {

                    slidesSubsystem.setJoint(0.7);
                    slidesSubsystem.runArmToPos(500,1);
                    slidesSubsystem.setSlides(0);
                    slidesSubsystem.spin(0.75);

                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-35,47),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-50,57),Math.toRadians(90)) //park in obs
                */

                .build();




        //init
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