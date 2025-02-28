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
public class ObsAutoNoPreload extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmSub slidesSubsystem = new ArmSub(hardwareMap, telemetry);
        Pose2d startPos = new Pose2d(-21, 63.5, Math.toRadians(0));
        drive.setPoseEstimate(startPos);


        TrajectorySequence test = drive.trajectorySequenceBuilder(startPos)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-26,35),Math.toRadians(270)) // side of sub
                .splineToConstantHeading(new Vector2d(-27,14),Math.toRadians(270)) //crosses sub leg
                .splineToConstantHeading(new Vector2d(-36,12),Math.toRadians(180)) // back to line up with sample to push

                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(35)))) // slow speed for plowing 1st sample in
                .splineToConstantHeading(new Vector2d(-37,53),Math.toRadians(90)) // pushes 1st sample into obs

                .resetConstraints()
                .splineToConstantHeading(new Vector2d(-45,48),Math.toRadians(270)) //moves back
                .splineToSplineHeading(new Pose2d(-40,32,Math.toRadians(0)),Math.toRadians(270)) //goes forward ish to halfway to 2nd
                .splineToConstantHeading(new Vector2d(-38,14),Math.toRadians(270)) //line up with 2nd
                .splineToConstantHeading(new Vector2d(-44,12), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-45,53),Math.toRadians(115)) //pushes 2nd in

                /*.splineToConstantHeading(new Vector2d(-42,48),Math.toRadians(270)) //moves back
                .splineToSplineHeading(new Pose2d(-38,32,Math.toRadians(0)),Math.toRadians(270)) //goes forward ish to halfway to 2nd
                .splineToConstantHeading(new Vector2d(-33,14),Math.toRadians(270)) //line up with 3nd
                .splineToConstantHeading(new Vector2d(-48,12), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-53,53),Math.toRadians(115)) //pushes 3nd in

                 */


                .splineToSplineHeading(new Pose2d(-45,52,Math.toRadians(90)),Math.toRadians(270)) // lines up with 1st specimen in obs
                .splineToConstantHeading(new Vector2d(-45,54),Math.toRadians(90)) // lines up with 1st specimen in obs
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(10)))) // slow speed for plowing 1st sample in
                .splineToConstantHeading(new Vector2d(-45,58.75),Math.toRadians(90)) // lines up with 1st specimen in obs
                .resetConstraints()

                .addTemporalMarker(3,() -> {
                    slidesSubsystem.clawOpen();
                    slidesSubsystem.setJoint(0.4);
                    slidesSubsystem.runArmToPos(500,1);
                    slidesSubsystem.setSlides(0);
                    slidesSubsystem.spin(0.75);
                })
                .waitSeconds(0.4)

                .addTemporalMarker(8,() -> {
                    slidesSubsystem.clawClose();
                })
                .addTemporalMarker(8.3,() -> {
                    slidesSubsystem.runArmToPos(2050,0.35);
                    slidesSubsystem.setJoint(0);
                    slidesSubsystem.spin(0.05);
                })
                .addTemporalMarker(8.5,() -> {
                    slidesSubsystem.setSlides(1400);
                })
                .resetConstraints()
                .splineToConstantHeading(new Vector2d(-15,60),Math.toRadians(0)) // to sub with 1st
                .splineToConstantHeading(new Vector2d(-10,46),Math.toRadians(0))  // to sub still
                //.waitSeconds(1)
                .addTemporalMarker(10.65,() -> {
                    slidesSubsystem.runArmToPos(2400,0.75);
                    slidesSubsystem.setSlides(325);

                })
                .addTemporalMarker(11.15,() -> {
                    slidesSubsystem.clawOpen();

                })
                .addTemporalMarker(11.3,() -> {

                    slidesSubsystem.setJoint(0.4);
                    slidesSubsystem.runArmToPos(500,1);
                    slidesSubsystem.setSlides(0);
                    slidesSubsystem.spin(0.75);

                })
                .addTemporalMarker(13,() -> {
                    slidesSubsystem.clawClose();
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-45,54),Math.toRadians(90)) // lines up with 2nd specimen in obs
                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(17)))) // slow speed for plowing 1st sample in
                .splineToConstantHeading(new Vector2d(-45,58.5),Math.toRadians(90))
                .resetConstraints()


                .waitSeconds(0.4)

                .addTemporalMarker(13.3,() -> {
                    slidesSubsystem.runArmToPos(2050,0.35);
                    slidesSubsystem.setJoint(0);
                    slidesSubsystem.spin(0.05);
                })
                .addTemporalMarker(13.5,() -> {
                    slidesSubsystem.setSlides(1400);
                })
                .setReversed(true)

                .splineToConstantHeading(new Vector2d(-45, 55),Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-20,57),Math.toRadians(0)) // to sub with 2nd one
                .splineToConstantHeading(new Vector2d(-10,55), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-6,45),Math.toRadians(270))  // to sub still
                .addTemporalMarker(15.6,() -> {
                    slidesSubsystem.runArmToPos(2400,0.75);
                    slidesSubsystem.setSlides(325);

                })
                .addTemporalMarker(16.1,() -> {
                    slidesSubsystem.clawOpen();

                })
                .addTemporalMarker(16.4,() -> {

                    slidesSubsystem.setJoint(0.4);
                    slidesSubsystem.runArmToPos(500,1);
                    slidesSubsystem.setSlides(0);
                    slidesSubsystem.spin(0.75);

                })
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