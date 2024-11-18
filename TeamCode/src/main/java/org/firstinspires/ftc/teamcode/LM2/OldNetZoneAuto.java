package org.firstinspires.ftc.teamcode.LM2;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
@Disabled
@Autonomous
@Config
public class OldNetZoneAuto extends LinearOpMode {
    public static double initX = 64, initY = -38, toX = 48, toY = 48;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        LM2SlidesSubsystem slidesSubsystem = new LM2SlidesSubsystem(hardwareMap, telemetry);

        drive.setPoseEstimate(new Pose2d(38, 64, 0));



        Trajectory toBasket = drive.trajectoryBuilder(new Pose2d(38, 64, 0))
                .lineToSplineHeading(new Pose2d(52, 48, Math.toRadians(45)))
                .addTemporalMarker(0.1, () -> {
                    slidesSubsystem.setSlidesJointPos(2000,2);
                    slidesSubsystem.update();

                })
                .addTemporalMarker(15, () -> {
                    slidesSubsystem.setSlides(4800);
                    slidesSubsystem.setJointPos(0.7);
                })

                .build();


        Trajectory deliverOne = drive.trajectoryBuilder(toBasket.end())
                .forward(7)
                .addTemporalMarker(10, () -> {
                    slidesSubsystem.spinnyDeliver();
                })
                .addTemporalMarker(14, () -> {
                    slidesSubsystem.turnOffSpinny();
                })
                .build();

        slidesSubsystem.setJointPos(0.15);

        waitForStart();
        drive.followTrajectory(toBasket);
        drive.followTrajectory(deliverOne);

        // Continuous update for SlidesSubsystem
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            slidesSubsystem.update();
            telemetry.update();
            break;

        }

    }
}