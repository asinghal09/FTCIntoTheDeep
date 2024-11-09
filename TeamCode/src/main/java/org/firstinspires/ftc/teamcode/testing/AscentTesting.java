package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp
public class AscentTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftWheel = hardwareMap.get(DcMotor.class,"leftWheel");
        DcMotor rightWheel = hardwareMap.get(DcMotor.class, "rightWheel");
        leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor hook1 = hardwareMap.get(DcMotor.class, "hook1");
        DcMotor hook2 = hardwareMap.get(DcMotor.class, "hook2");
        hook1.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            double drive = gamepad1.left_stick_y;
            leftWheel.setPower(drive);
            rightWheel.setPower(drive);

            hook1.setPower(gamepad1.right_stick_y);
            hook2.setPower(gamepad1.right_stick_y);
        }
    }
}
