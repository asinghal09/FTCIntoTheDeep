package org.firstinspires.ftc.teamcode.LM2;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.opencv.core.Mat;

@Autonomous
@Config
public class NetZoneAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        SlidesSubsystem slidesSubsystem = new SlidesSubsystem(hardwareMap, telemetry);

        drive.setPoseEstimate(new Pose2d(6, 64, Math.toRadians(270)));



        Trajectory toSubmersible = drive.trajectoryBuilder(new Pose2d(6, 64, Math.toRadians(270)))
                .forward(15)
                .addTemporalMarker(0.1, () -> {
                    slidesSubsystem.setSlidesJointPos(1650,2);

                })
                .addTemporalMarker(3, () -> {
                    slidesSubsystem.setSlides(500);
                    slidesSubsystem.setJointPos(0.625);
                })

                .build();

        Trajectory deliverOne = drive.trajectoryBuilder(toSubmersible.end())
                .forward(12)
                .addTemporalMarker(6, ()->{
                    slidesSubsystem.setSlidesJointPos(1000,6);
                })
                .build();
        Trajectory backUp = drive.trajectoryBuilder(deliverOne.end())
                .back(8)
                .addTemporalMarker(11, ()->{
                    slidesSubsystem.setJointPos(.7);
                    slidesSubsystem.setSlidesJointPos(400,4);
                    slidesSubsystem.setSlides(0);
                })
                .build();
        Trajectory pickOne = drive.trajectoryBuilder(backUp.end())
                .strafeTo(new Vector2d(48,39))
                .build();
        Trajectory basketLineup = drive.trajectoryBuilder(pickOne.end())
                .lineToSplineHeading(new Pose2d(52, 48, Math.toRadians(45)))
                .addTemporalMarker(12, () -> {
                    slidesSubsystem.setSlidesJointPos(2000,2);

                })
                .addTemporalMarker(15, () -> {
                    slidesSubsystem.setSlides(4700);
                    slidesSubsystem.setJointPos(0.7);


                })
                .build();
        Trajectory deliver = drive.trajectoryBuilder(basketLineup.end())
                .forward(7)
                //.addTemporalMarker(16, () -> {
                  //  slidesSubsystem.spinnyDeliver();
                //})
                //.addTemporalMarker(18, () -> {
                    //slidesSubsystem.turnOffSpinny();
               //})
                .build();
        Trajectory pickTwo = drive.trajectoryBuilder(deliver.end())
                .lineToSplineHeading(new Pose2d(60, 39, Math.toRadians(45)))
                .addTemporalMarker(20,()->{
                    //slidesSubsystem.setSlidesJointPos(700);
                })
                .build();



        slidesSubsystem.setJointPos(0.15);

        waitForStart();
        drive.followTrajectory(toSubmersible);

        ElapsedTime timer = new ElapsedTime();
        while(opModeIsActive() && timer.seconds() < 3){
            slidesSubsystem.update();
        }
        drive.followTrajectory(deliverOne);

        timer.reset();
        while(opModeIsActive() && timer.seconds() < 3){
            slidesSubsystem.update();

        }
        slidesSubsystem.spinnyDeliver();
        drive.followTrajectory(backUp);
        drive.followTrajectory(pickOne);

        timer.reset();
        while(opModeIsActive() && timer.seconds() < 1.5){
            slidesSubsystem.update();
        }
        slidesSubsystem.turnOffSpinny();
        drive.followTrajectory(basketLineup);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 2){
            slidesSubsystem.update();
        }
        drive.followTrajectory(deliver);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 6){
            slidesSubsystem.update();
        }
        drive.followTrajectory(pickTwo);
        drive.followTrajectory(basketLineup);
        drive.followTrajectory(deliver);




        // Continuous update for SlidesSubsystem
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            slidesSubsystem.update();
            telemetry.update();

        }

    }
}