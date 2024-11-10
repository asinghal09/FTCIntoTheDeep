package org.firstinspires.ftc.teamcode.LM2;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
                    slidesSubsystem.runArmToPos(1650,0.75);
                    //slidesSubsystem.setSlidesJointPos(1650,2);

                })
                .addTemporalMarker(0.25, () -> {
                    slidesSubsystem.setSlides(500);
                    slidesSubsystem.setJointPos(0.625);
                })

                .build();

        Trajectory deliverOne = drive.trajectoryBuilder(toSubmersible.end())    //drives up to submersible
                .forward(12)
                //.addTemporalMarker(6.75, ()->{
                    //slidesSubsystem.setSlidesJointPos(1000,6); //brings down arm to push onto chamber
                //})
                .build();
        Trajectory backUp = drive.trajectoryBuilder(deliverOne.end())       //backs up from sub after delivering
                .back(10)
                .addTemporalMarker(11, ()->{
                    slidesSubsystem.setJointPos(0.5);                           //intake pos specimen
                    slidesSubsystem.runArmToPos(600,1);
                    //slidesSubsystem.setSlidesJointPos(335,4);
                    slidesSubsystem.setSlides(0);
                })
                .build();
        Trajectory splineToObs = drive.trajectoryBuilder(backUp.end())         //strafes to pick up 1st sample
                .splineToLinearHeading(new Pose2d(-46.5,53,Math.toRadians(90)), Math.toRadians(180))
                .addSpatialMarker(new Vector2d(-46, 53), () -> {
                    slidesSubsystem.spinnyIntake();
                })
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
                .forward(11)
                .build();
        Trajectory backUp2 = drive.trajectoryBuilder(forward.end())
                .back(10)
                .build();

        Trajectory push = drive.trajectoryBuilder(backUp2.end())
                .strafeRight(14)
                .splineToSplineHeading(new Pose2d(-39,12,Math.toRadians(0)),Math.toRadians(270))
                .build();
        Trajectory backPush = drive.trajectoryBuilder(push.end())
                .back(9.5)
                .build();
        Trajectory strafe = drive.trajectoryBuilder(backPush.end())
                .strafeLeft(48)
                .build();




        //init
        slidesSubsystem.setJointPos(0.125);

        waitForStart();

        //beginning of auto
        drive.followTrajectory(toSubmersible);

        ElapsedTime timer = new ElapsedTime();
        while(opModeIsActive() && timer.seconds() < 0.75){
            slidesSubsystem.update();
        }
        drive.followTrajectory(deliverOne);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.5){
            slidesSubsystem.update();
        }
        slidesSubsystem.runArmToPos(1200,0.4);
        //slidesSubsystem.setSlidesJointPos(1000,6);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 1){
            slidesSubsystem.update();
        }
        slidesSubsystem.spinnyDeliver();
        drive.followTrajectory(backUp);
        slidesSubsystem.turnOffSpinny();
        drive.followTrajectory(splineToObs);
        drive.followTrajectory(pickOne);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.2){
            slidesSubsystem.update();
        }
        slidesSubsystem.turnOffSpinny();
        slidesSubsystem.runArmToPos(1650,1);
        slidesSubsystem.setJointPos(0.625);
        slidesSubsystem.setSlides(500);
        drive.followTrajectory(backUpFromPerimeter);
        drive.followTrajectory(splineToSub);
        drive.followTrajectory(forward);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.5){
            slidesSubsystem.update();
        }
        slidesSubsystem.runArmToPos(1200,0.4);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 1){
            slidesSubsystem.update();
        }
        slidesSubsystem.spinnyDeliver();
        drive.followTrajectory(backUp2);
        slidesSubsystem.turnOffSpinny();
        slidesSubsystem.runArmToPos(0,1);
        slidesSubsystem.setSlides(0);
        slidesSubsystem.setJointPos(0.15);
        drive.followTrajectory(push);
        drive.followTrajectory(backPush);
        drive.followTrajectory(strafe);


        // Continuous update for SlidesSubsystem
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            //slidesSubsystem.update();
            telemetry.update();

        }

    }
}