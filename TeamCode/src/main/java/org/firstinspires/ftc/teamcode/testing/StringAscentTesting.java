package org.firstinspires.ftc.teamcode.testing;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp
public class StringAscentTesting extends LinearOpMode {  
    DcMotor ascent;
    @Override
    public void runOpMode() throws InterruptedException {
        ascent = hardwareMap.get(DcMotor.class, "ascent");

        waitForStart();
        while (opModeIsActive()){
            ascent.setPower(gamepad1.left_stick_y);

        }
    }
}
