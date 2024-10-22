package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous
public class Auto extends LinearOpMode {

    private FtcDashboard dashboard;
    boolean blue;
    boolean netZone;

    public void runOpMode() throws InterruptedException{

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        AutoMethods robot = new AutoMethods(frontLeft, frontRight, backLeft, backRight);
        AutoSelector selector = new AutoSelector();




        waitForStart();

       // if (blue && netZone){   //blue net zone
            robot.driveForward(0.5, 200);
            robot.turnInPlace(270);
            robot.driveForward(0.5, 900);
            sleep(300);
            robot.driveBackward(0.5, 450);
            robot.turnInPlace(90);
            robot.driveForward(0.5,700);
            robot.strafe(true, 0.5, 350);
            robot.turnInPlace(220);
            robot.driveForward(0.3, 1800);
       // } else if (blue && !netZone){   //blue observation


        //} else if (!blue && netZone){   // red net zone


        //}else if (!blue && !netZone) {  //red observation

       // }


        //while (!isStopRequested() && opModeIsActive()){




    }

}
