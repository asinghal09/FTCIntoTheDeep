package org.firstinspires.ftc.teamcode.testing;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LM3.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
@Disabled
@Autonomous
@Config
public class RRTurnTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        SlidesSubsystem slidesSubsystem = new SlidesSubsystem(hardwareMap, telemetry);

        drive.setPoseEstimate(new Pose2d(6, 64, Math.toRadians(270)));


        Trajectory forward = drive.trajectoryBuilder(new Pose2d(6, 64, Math.toRadians(270)))
                .forward(15)
                .build();

        Trajectory turn = drive.trajectoryBuilder(forward.end())
                .lineToSplineHeading(new Pose2d(6,44,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(6,44,Math.toRadians(65)))
                .build();


        waitForStart();
        //drive.followTrajectory(forward);
        drive.followTrajectory(turn);
    }
}

