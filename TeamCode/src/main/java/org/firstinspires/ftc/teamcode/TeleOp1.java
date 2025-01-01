package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class TeleOp1 extends OpMode{
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;

    @Override
    public void init() {
        //initialize
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //set direction
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        double turn = gamepad1.left_stick_x; //forward and backward
        double drive = -gamepad1.right_stick_y; //turn right and left

        //calculate power
        double frontLeftPower = drive + turn;
        double frontRightPower = drive - turn;
        double backLeftPower = drive + turn;
        double backRightPower = drive - turn;

        //set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        //display telemetry data
        telemetry.addData("Motors", "FL: %.2f, FR: %.2f, BL:%.2f, BR: %.2f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
}
