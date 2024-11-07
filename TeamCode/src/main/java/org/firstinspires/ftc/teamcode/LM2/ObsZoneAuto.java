package org.firstinspires.ftc.teamcode.LM2;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.opencv.core.Mat;

@Autonomous
@Config

public class ObsZoneAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        SlidesSubsystem slidesSubsystem = new SlidesSubsystem(hardwareMap, telemetry);

        drive.setPoseEstimate(new Pose2d(-6, 64, Math.toRadians(270)));



        Trajectory toSubmersible = drive.trajectoryBuilder(new Pose2d(-6, 64, Math.toRadians(270)))
                .forward(15)
                .addTemporalMarker(0.1, () -> {
                    slidesSubsystem.setSlidesJointPos(1600);
                    //slidesSubsystem.update();

                })
                .addTemporalMarker(3, () -> {
                    slidesSubsystem.setSlides(150);
                    slidesSubsystem.setJointPos(0.625);
                })

                .build();


        Trajectory deliverOne = drive.trajectoryBuilder(toSubmersible.end())
                .forward(11.25)
                .addTemporalMarker(7, ()->{
                    slidesSubsystem.setSlidesJointPos(1000);
                }).addTemporalMarker(8, ()->{
                    slidesSubsystem.wheelsDeliver();
                })
                .addTemporalMarker(9, ()->{
                    slidesSubsystem.turnOffWheels();
                })

                .build();
        Trajectory pickOne = drive.trajectoryBuilder(deliverOne.end())
                .back(8)
                .strafeTo(new Vector2d(-30,40))
                .build();



        slidesSubsystem.setJointPos(0.15);


        waitForStart();
        drive.followTrajectory(toSubmersible);

        ElapsedTime timer = new ElapsedTime();
        while(opModeIsActive() && timer.seconds() < 3){
            slidesSubsystem.update();
        }
        drive.followTrajectory(deliverOne);


        // Continuous update for SlidesSubsystem
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            slidesSubsystem.update();
            telemetry.update();

        }


    }
}


