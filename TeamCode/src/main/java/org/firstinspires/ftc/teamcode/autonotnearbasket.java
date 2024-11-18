package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous
public class autonotnearbasket extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor arm;

    int frontLeftPos;
    int frontRightPos;
    int backLeftPos;
    int backRightPos;

    private static final double Kp = 0.05;
    private static final double Ki = 0.001;
    private static final double Kd = 0.1;

    private double previousError = 0.0;
    private double integral = 0.0;

    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        arm = hardwareMap.get(DcMotor.class, "arm");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftPos = 0;
        frontRightPos = 0;
        backLeftPos = 0;
        backRightPos = 0;


        waitForStart();

        int targetPosition = 1000;

        //850 Ticks for 90 degrees
        //9.44 Tick/Degree
        //14.38 Tick/cm
        // Get current position of both motors
        while (opModeIsActive()) {

            // Get current position of both motors
            int currentPosition = (arm.getCurrentPosition());

            // Compute the error
            double error = targetPosition - currentPosition;

            // Compute the integral (sum of errors)
            integral += error;

            // Compute the derivative (change in error)
            double derivative = error - previousError;

            // Compute the PID output (control signal)
            double pidOutput = Kp * error + Ki * integral + Kd * derivative;

            // Update the previous error for the next iteration
            previousError = error;

            // Set motor power based on the PID output (use same value for both motors)
            arm.setPower(pidOutput);

            // Telemetry to show the current status
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("PID Output", pidOutput);
            telemetry.update();

            // If we've reached the target, break out of the loop
            if (Math.abs(targetPosition - currentPosition) < 10) {
                break;
            }

            // Add some delay to reduce the number of iterations and prevent excessive motor commands
            sleep(50);
        }

        // Stop the motors once the target is reached
        arm.setPower(0);


        drive(992.22, 992.22, 992.22, 992.22, 0.3);
        sleep(300);
        drive(-143.8, -143.9, -143.8, -143.8, 0.3);
        drive(850,-850, -850, 850, 0.3);
        drive(1610.56, 1610.56, 1610.56, 1610.56, 0.3);
        drive(850, -850, -850, 850, 0.3);
        drive(600, 600, 600, 600, 0.3);
        drive(-600, -600, -600, -600, 0.3);
        drive(850, -850, -850, 850, 0.3);
        drive(1610.56, 1610.56, 1610.56, 1610.56, 0.3);
        //turn right 90 degrees
        drive(850, -850, -850, 850, 0.3);
        drive(143.8, 143.8, 143.8, 143.8, 0.3);
        sleep(300);
        //back
        drive(-143.8, -143.8, -143.8, -143.8, 0.3);
        drive(850, -850, -850, 850, 0.3);
        drive(1000, 1000, 1000, 1000, 0.3);
        //strafe left
        drive(1006.6, -1006.6, 1006.6, -1006.6, 0.3);
        //forward
        drive(575.2, 575.2, 575.2, 575.2, 0.3);
        //strafe right
        drive(-1653.7, 1653.7, -1653.7, 1653.7, 0.3);


    }

    private void drive(double bLeftTarget, double bRightTarget, double fRightTarget, double fLeftTarget, double speed ) {
        frontLeftPos += fLeftTarget;
        frontRightPos += fRightTarget;
        backLeftPos += bLeftTarget;
        backRightPos += bRightTarget;

        frontLeft.setTargetPosition(frontLeftPos);
        frontRight.setTargetPosition(frontRightPos);
        backLeft.setTargetPosition(backLeftPos);
        backRight.setTargetPosition(backRightPos);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while(opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
            idle();
            // sleep(1000);

            //frontLeft.setPower(0);
            //  frontRight.setPower(0);
            //  backLeft.setPower(0);
            //  backRight.setPower(0);
        }
    }
}
