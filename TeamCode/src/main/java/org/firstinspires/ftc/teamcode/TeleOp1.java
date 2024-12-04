package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class TeleOp1 extends OpMode{
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;

    @Override
    public void init() {
        //initialize
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        //set direction
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y; //forward and backward
        double turn = gamepad1.right_stick_x; //turn right and left

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
