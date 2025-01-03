package org.firstinspires.ftc.teamcode.LM4;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;


@Autonomous
@Config
public class ObsAutoNew extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmSub slidesSubsystem = new ArmSub(hardwareMap, telemetry);
        Pose2d startPos = new Pose2d(-10, 63.5, Math.toRadians(270));
        drive.setPoseEstimate(startPos);


        TrajectorySequence test = drive.trajectorySequenceBuilder(startPos)
                .splineToConstantHeading(new Vector2d(-11, 35), Math.toRadians(270)) // to sub
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
                .addTemporalMarker(3,() -> {
                    slidesSubsystem.clawOpen();
                })
                .waitSeconds(0.9)
                .splineToConstantHeading(new Vector2d(-14,45), Math.toRadians(90)) //back up from sub
                .splineToSplineHeading(new Pose2d(-35,62.5, Math.toRadians(180)), Math.toRadians(180)) //to obs zone, lines up with specimen
                .splineToConstantHeading(new Vector2d(-48.5,62.5),Math.toRadians(180)) //drives into specimen
                .waitSeconds(1.5)

                .addTemporalMarker(3.5,() -> {
                    slidesSubsystem.setSlides(1000);                        //picking up pos
                    slidesSubsystem.runArmToPos(100,1);
                })
                .addTemporalMarker(5.5,()-> {
                    slidesSubsystem.clawClose();
                })
                .addTemporalMarker(6,()-> {
                    slidesSubsystem.runArmToPos(900,1);
                })
                .addTemporalMarker(6.25,()->{
                    slidesSubsystem.setSlides(2570);
                })
                .splineToSplineHeading(new Pose2d(-6, 35, Math.toRadians(270)), Math.toRadians(270)) //to sub with 2nd specimen

                .addTemporalMarker(9,() -> {
                    slidesSubsystem.runArmToPos(1100,0.5);
                })
                .addTemporalMarker(10.5,() -> {
                    slidesSubsystem.clawOpen();
                })
                .waitSeconds(1.5)
                .splineToConstantHeading(new Vector2d(-8,45), Math.toRadians(90)) //back up from sub
                .build();

        slidesSubsystem.clawClose();

        waitForStart();
        if (!isStopRequested()){
            drive.followTrajectorySequence(test);
        }
    }
}