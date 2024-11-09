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
                .forward(15)                //brings robot to the submersible for 1st specimen
                .addTemporalMarker(0.1, () -> {
                    slidesSubsystem.runArmToPos(1650,0.5);
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
                .back(8)
                .addTemporalMarker(11, ()->{
                    slidesSubsystem.setJointPos(.71);                            //intake pos
                    slidesSubsystem.runArmToPos(550,0.25);
                    //slidesSubsystem.setSlidesJointPos(335,4);
                    slidesSubsystem.setSlides(0);
                })
                .build();
        Trajectory pickOne = drive.trajectoryBuilder(backUp.end())         //strafes to pick up 1st sample
                .strafeTo(new Vector2d(48,41))
                .addSpatialMarker(new Vector2d(20, 42), () -> {
                slidesSubsystem.spinnyIntake();
                })
                .build();
        Trajectory basketLineup = drive.trajectoryBuilder(pickOne.end()) //goes to basket
                .lineToSplineHeading(new Pose2d(52, 48, Math.toRadians(55)))

                .addTemporalMarker(11.5, () -> {
                    //slidesSubsystem.setSlidesJointPos(2200,2);
                    slidesSubsystem.runArmToPos(2050,0.5);

                })
                .addTemporalMarker(13, () -> {
                    slidesSubsystem.setSlides(5000);
                    slidesSubsystem.setJointPos(0.625);


                })
                .build();
        Trajectory deliver = drive.trajectoryBuilder(basketLineup.end())
                .forward(13.5)
                //.addTemporalMarker(16, () -> {
                  //  slidesSubsystem.spinnyDeliver();
                //})
                //.addTemporalMarker(18, () -> {
                    //slidesSubsystem.turnOffSpinny();
               //})
                .build();
        Trajectory backUp2 = drive.trajectoryBuilder(deliver.end())
                .back(12)
                .build();
        Trajectory pickTwo = drive.trajectoryBuilder(backUp2.end().plus(new Pose2d(0,0,Math.toRadians(220))))
                .lineToSplineHeading(new Pose2d(58.5, 42, Math.toRadians(270)))
                .build();
        Trajectory basketLineup2 = drive.trajectoryBuilder(pickTwo.end()) //goes to basket
                .lineToSplineHeading(new Pose2d(52, 48, Math.toRadians(55)))
                        .build();



        slidesSubsystem.setJointPos(0.125);



        waitForStart();
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
        slidesSubsystem.runArmToPos(1150,0.2);
        //slidesSubsystem.setSlidesJointPos(1000,6);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 1){
            slidesSubsystem.update();
        }
        slidesSubsystem.spinnyDeliver();
        drive.followTrajectory(backUp);
        drive.followTrajectory(pickOne);

        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.25){
            slidesSubsystem.update();
        }
        slidesSubsystem.runArmToPos(350,0.25);

        timer.reset();
        while(opModeIsActive() && timer.seconds() < 1){
            slidesSubsystem.update();
        }

        slidesSubsystem.turnOffSpinny();
        slidesSubsystem.runArmToPos(700,1);
        //slidesSubsystem.setSlidesJointPos(700,2);
        drive.followTrajectory(basketLineup);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 3){
            slidesSubsystem.update();
        }
        drive.followTrajectory(deliver);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.5){
            slidesSubsystem.update();
        }
        slidesSubsystem.spinnyDeliver();
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 1.5){
            slidesSubsystem.update();
        }
        drive.followTrajectory(backUp2);
        slidesSubsystem.setSlides(0);
        slidesSubsystem.turnOffSpinny();
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.5){
            slidesSubsystem.update();
        }
        slidesSubsystem.runArmToPos(550,0.75);
        //slidesSubsystem.setSlidesJointPos(375,2);

        timer.reset();
        while(opModeIsActive() && timer.seconds() < 1.5){
            slidesSubsystem.update();
        }
        drive.turn(Math.toRadians(220));
        drive.followTrajectory(pickTwo);
        slidesSubsystem.spinnyIntake();
        slidesSubsystem.setJointPos(0.71);

        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.5){
            slidesSubsystem.update();
        }
        slidesSubsystem.runArmToPos(350,0.5);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 1){
            slidesSubsystem.update();
        }
        slidesSubsystem.turnOffSpinny();
        slidesSubsystem.runArmToPos(2000,0.5);
        slidesSubsystem.setSlides(5000);
        slidesSubsystem.setJointPos(0.625);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 1){
            slidesSubsystem.update();
        }
        drive.followTrajectory(basketLineup2);
        drive.followTrajectory(deliver);
        timer.reset();
        while(opModeIsActive() && timer.seconds() < 0.5){
            slidesSubsystem.update();
        }
        slidesSubsystem.spinnyDeliver();



        //drive.followTrajectory(basketLineup);
        //drive.followTrajectory(deliver);




        // Continuous update for SlidesSubsystem
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            //slidesSubsystem.update();
            telemetry.update();

        }

    }
}