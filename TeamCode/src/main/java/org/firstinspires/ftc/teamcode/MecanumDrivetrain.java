package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumDrivetrain extends LinearOpMode{
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;

    public void runOpMode(){

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        while(opModeIsActive()){

            double driveLeft = gamepad1.left_stick_y/2;
            double driveRight = gamepad1.right_stick_y/2;
            double strafe = gamepad1.right_stick_x/2;



            frontLeft.setPower(driveLeft-strafe);
            frontRight.setPower(driveRight+strafe);
            backLeft.setPower(driveLeft+strafe);
            backRight.setPower(driveRight-strafe);



            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.addData("Front Right Power", frontRight.getPower());
            telemetry.addData("Back Left Power", backLeft.getPower());
            telemetry.addData("Back Right Power", backRight.getPower());
            telemetry.update();




        }

    }


}
