package org.firstinspires.ftc.teamcode.LM3;
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
public class NetZoneAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        SlidesSubsystem slidesSubsystem = new SlidesSubsystem(hardwareMap, telemetry);

        drive.setPoseEstimate(new Pose2d(6, 64, Math.toRadians(270)));


        Trajectory toSubmersible = drive.trajectoryBuilder(new Pose2d(6, 64, Math.toRadians(270)))
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
        Trajectory pickOne = drive.trajectoryBuilder(backUp.end())         //strafes to pick up 1st sample
                .strafeTo(new Vector2d(50,48))
                .addSpatialMarker(new Vector2d(21, 39), () -> {
                slidesSubsystem.runArmToPos(325,1);
                })
                .build();
        Trajectory basketLineup = drive.trajectoryBuilder(pickOne.end()) //goes to basket
                .lineToSplineHeading(new Pose2d(54, 50, Math.toRadians(50)))
                .build();
        Trajectory deliver = drive.trajectoryBuilder(basketLineup.end())
                .forward(13)
                .build();
        Trajectory backUp2 = drive.trajectoryBuilder(deliver.end())
                .back(12)
                .build();


        Trajectory turnToTwo = drive.trajectoryBuilder(backUp2.end())
                .lineToSplineHeading(new Pose2d(50,48, Math.toRadians(-64)))
                .build();

        Trajectory backToTwo = drive.trajectoryBuilder(backUp2.end())
                .lineToSplineHeading(new Pose2d(40,14,Math.toRadians(90)))
                .build();

        Trajectory strafeToTwo = drive.trajectoryBuilder(backToTwo.end())
                .strafeTo(new Vector2d(61,18))
                .build();

        Trajectory spline = drive.trajectoryBuilder(backUp2.end(), true)
                .splineToSplineHeading(new Pose2d(40,12,Math.toRadians(90)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(58,18, Math.toRadians(90)), Math.toRadians(0))
                .build();


        Trajectory pickTwo = drive.trajectoryBuilder(backUp2.end().plus(new Pose2d(0,0,Math.toRadians(220))))
                .lineToSplineHeading(new Pose2d(58.5, 39, Math.toRadians(270)))
                .build();
        Trajectory basketLineup2 = drive.trajectoryBuilder(turnToTwo.end()) //goes to basket
                .lineToSplineHeading(new Pose2d(51, 51, Math.toRadians(45)))
                .build();

        Trajectory deliver2 = drive.trajectoryBuilder(basketLineup2.end())
                .forward(11)
                .build();
        Trajectory backUp3 = drive.trajectoryBuilder(deliver2.end())
                .back(20)
                .build();
        Trajectory levelOneAscent = drive.trajectoryBuilder(backUp2.end())
                .lineToSplineHeading(new Pose2d(20,12,Math.toRadians(0)))
                .build();

        Trajectory push3 = drive.trajectoryBuilder(backUp3.end())
                .lineToSplineHeading(new Pose2d(63,18,Math.toRadians(180)))
                .build();
        Trajectory strafe3In = drive.trajectoryBuilder(push3.end())
                .strafeRight(38)
                .build();


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

        drive.followTrajectory(pickOne);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.025){
            slidesSubsystem.update();
        }
        slidesSubsystem.runArmToPos(0,0.5);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 1){
            slidesSubsystem.update();
        }
        slidesSubsystem.clawClose();
        slidesSubsystem.runArmToPos(1650,1);

        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.1){
            slidesSubsystem.update();
        }
        slidesSubsystem.setSlides(0);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.1){
            slidesSubsystem.update();
        }
        drive.followTrajectory(basketLineup);

        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.15){
            slidesSubsystem.update();
        }

        slidesSubsystem.setSlides(4100);

        timer.reset();
        while(opModeIsActive() && timer.seconds() < 1){
            slidesSubsystem.update();
        }
        drive.followTrajectory(deliver);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.5){
            slidesSubsystem.update();
        }
        slidesSubsystem.clawOpen();

        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.25){
            slidesSubsystem.update();
        }
        drive.followTrajectory(backUp2);
        slidesSubsystem.setSlides(500);
        slidesSubsystem.runArmToPos(450,0.4);

        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.25){
            slidesSubsystem.update();
        }
        drive.followTrajectory(turnToTwo);
        slidesSubsystem.setSlides(1775);
        slidesSubsystem.runArmToPos(175,0.5);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 1.2){
            slidesSubsystem.update();
        }
        slidesSubsystem.clawClose();
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.2){
            slidesSubsystem.update();
        }
        slidesSubsystem.runArmToPos(1650,1);
        slidesSubsystem.setSlides(0);

        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.5){
            slidesSubsystem.update();
        }
        drive.followTrajectory(basketLineup2);

        slidesSubsystem.setSlides(4100);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 1.3){
            slidesSubsystem.update();
        }
        drive.followTrajectory(deliver2);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.5){
            slidesSubsystem.update();
        }
        slidesSubsystem.clawOpen();
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.5){
            slidesSubsystem.update();
        }
        drive.followTrajectory(backUp3);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.25){
            slidesSubsystem.update();
        }
        slidesSubsystem.setSlides(0);
        slidesSubsystem.runArmToPos(0,0.5);



        // Continuous update for SlidesSubsystem
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            //slidesSubsystem.update();
            telemetry.update();

        }

    }
}