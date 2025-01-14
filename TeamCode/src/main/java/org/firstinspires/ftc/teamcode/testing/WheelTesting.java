package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp
public class WheelTesting extends LinearOpMode {


    DcMotor frontLeft, frontRight, backLeft, backRight;
    @Override
    public void runOpMode() throws InterruptedException {


        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        frontLeft.setPower(0.75);
        sleep(5000);
        frontLeft.setPower(0);

        frontRight.setPower(0.75);
        sleep(5000);
        frontRight.setPower(0);

        backLeft.setPower(0.75);
        sleep(5000);
        backLeft.setPower(0);

        backRight.setPower(0.75);
        sleep(5000);
        backRight.setPower(0);



    }
}
