package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class SpinnyIntakeTest extends LinearOpMode {

    DcMotor arm;
    CRServo wheels;
    boolean isIntaking = false;
    boolean previousAState = false;
    boolean isDelivering = false;
    boolean previousBState = false;
    int armPos = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        wheels = hardwareMap.get(CRServo.class, "wheels");
        arm = hardwareMap.get(DcMotor.class, "arm");
        wheels.setDirection(DcMotorSimple.Direction.REVERSE);

        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()){
            /*
            armPos += (int) (gamepad1.right_stick_y * 10);
            armPos = Math.min(armPos, arm.getCurrentPosition() + 100); // Limit max arm position change
            armPos = Math.max(armPos, arm.getCurrentPosition() - 100); // Limit min arm position change

            arm.setTargetPosition(armPos);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             */

            arm.setPower(gamepad1.right_stick_y);

            boolean currentAState = gamepad1.a;
            boolean currentBState = gamepad1.b;
            // Toggle spinning when 'A' button is pressed
            if (currentAState && !previousAState) {
                // Toggle the spinning state
                isIntaking = !isIntaking;

                // If spinning, set servo speed
                if (isIntaking) {
                    wheels.setPower(0.5);
                } else {       // If not spinning, stop the servo
                    wheels.setPower(0);
                }
            }
            // Update the previous state of the 'A' button
            previousAState = currentAState;

            if(currentBState && !previousBState) {
                isDelivering = !isDelivering;
                if (isDelivering){
                    wheels.setPower(-0.5);
                } else {
                    wheels.setPower(0);
                }
            }
            previousBState = currentBState;

            idle();

        }

    }
}
