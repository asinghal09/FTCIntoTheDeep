package org.firstinspires.ftc.teamcode.LM3;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LM2.LM2SlidesSubsystem;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous
@Config
public class ObsZoneAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        SlidesSubsystem slidesSubsystem = new SlidesSubsystem(hardwareMap, telemetry);

        drive.setPoseEstimate(new Pose2d(-8, 64, Math.toRadians(270)));



        Trajectory toSubmersible = drive.trajectoryBuilder(new Pose2d(-8, 64, Math.toRadians(270)))
                .forward(15)                //brings robot to the submersible for 1st specimen
                .addTemporalMarker(0.1, () -> {
                    slidesSubsystem.runArmToPos(900,1);
                    //slidesSubsystem.setSlidesJointPos(1650,2);

                })
                .addTemporalMarker(0.25, () -> {
                    slidesSubsystem.setSlides(1650);
                })

                .build();

        Trajectory deliverOne = drive.trajectoryBuilder(toSubmersible.end())    //drives up to submersible
                .forward(11.5)
                .build();
        Trajectory backUp = drive.trajectoryBuilder(deliverOne.end())       //backs up from sub after delivering
                .back(5)
                .build();
        Trajectory splineToObs = drive.trajectoryBuilder(backUp.end())         //strafes to pick up 1st sample
                .splineToLinearHeading(new Pose2d(-48,48,Math.toRadians(90)), Math.toRadians(180))
                .build();
        Trajectory pickOne = drive.trajectoryBuilder(splineToObs.end())
                .forward(2)
                .build();

        Trajectory backUpFromPerimeter = drive.trajectoryBuilder(pickOne.end())
                .back(4)
                .build();

        Trajectory splineToSub = drive.trajectoryBuilder(backUpFromPerimeter.end())         //strafes to pick up 1st sample
                .splineToLinearHeading(new Pose2d(-6,49,Math.toRadians(271)), Math.toRadians(180))
                .build();
        Trajectory forward = drive.trajectoryBuilder(splineToSub.end())
                .forward(11.5)
                .build();
        Trajectory backUp2 = drive.trajectoryBuilder(forward.end())
                .back(5)
                .build();

        Trajectory backFromSub2 = drive.trajectoryBuilder(backUp2.end())
                .back(5)
                .build();

        Trajectory push = drive.trajectoryBuilder(backFromSub2.end())
                .strafeRight(14)
                .splineToSplineHeading(new Pose2d(-37,12,Math.toRadians(0)),Math.toRadians(270))
                .build();
        Trajectory backPush = drive.trajectoryBuilder(push.end())
                .back(9.5)
                .build();
        Trajectory strafe = drive.trajectoryBuilder(backPush.end())
                .strafeLeft(48)
                .build();

        Trajectory strafeRightTo2ndGround = drive.trajectoryBuilder(strafe.end())
                .strafeRight(48)
                .build();
        Trajectory backToStafe = drive.trajectoryBuilder(strafeRightTo2ndGround.end())
                .back(8)
                .build();
        Trajectory strafeToObs2 = drive.trajectoryBuilder(backToStafe.end())
                .strafeLeft(48)
                .build();



        //init
        slidesSubsystem.clawClose();

        waitForStart();
        drive.followTrajectory(toSubmersible);

        ElapsedTime timer = new ElapsedTime();
        while(opModeIsActive() && timer.seconds() < 0.2){
            slidesSubsystem.update();
        }
        drive.followTrajectory(deliverOne);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.3){
            slidesSubsystem.update();
        }
        slidesSubsystem.runArmToPos(1250,1);

        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.5){
            slidesSubsystem.update();
        }

        drive.followTrajectory(backUp);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.1){
            slidesSubsystem.update();
        }
        slidesSubsystem.clawOpen();
        slidesSubsystem.setSlides(1500);

        while(opModeIsActive() && timer.seconds() < 0.5){
            slidesSubsystem.update();
        }

        slidesSubsystem.runArmToPos(0,0.75);
        slidesSubsystem.setSlides(50);
        drive.followTrajectory(splineToObs);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.75){
            slidesSubsystem.update();
        }
        drive.followTrajectory(pickOne);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.4){
            slidesSubsystem.update();
        }

        slidesSubsystem.clawClose();
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.1){
            slidesSubsystem.update();
        }
        slidesSubsystem.runArmToPos(900,1);

        slidesSubsystem.setSlides(1650);
        drive.followTrajectory(backUpFromPerimeter);
        drive.followTrajectory(splineToSub);

        drive.followTrajectory(forward);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.5){
            slidesSubsystem.update();
        }
        slidesSubsystem.runArmToPos(1250,1);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.5){
            slidesSubsystem.update();
        }
        drive.followTrajectory(backUp2);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.1){
            slidesSubsystem.update();
        }
        slidesSubsystem.clawOpen();
        drive.followTrajectory(backFromSub2);
        slidesSubsystem.setSlides(0);
        slidesSubsystem.runArmToPos(0,0.75);
        drive.followTrajectory(push);
        drive.followTrajectory(backPush);
        drive.followTrajectory(strafe);
        drive.followTrajectory(strafeRightTo2ndGround);
        drive.followTrajectory(strafeToObs2);


        // Continuous update for SlidesSubsystem
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            //slidesSubsystem.update();
            telemetry.update();

        }
    }
}