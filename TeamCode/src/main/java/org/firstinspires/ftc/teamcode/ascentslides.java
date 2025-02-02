package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class ascentslides extends OpMode{
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor slideMotor;  // Single motor for slide

    // PID coefficients
    double kP = 0.01, kI = 0.0, kD = 0.0; // For slideMotor

    // Variables for PID
    double targetPosition = 0; // Target for slideMotor

    double integral = 0, lastError = 0;

    double armPowerScale = 0.5;  // Set to 50% power; adjust as needed

    @Override
    public void init() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        // Reset and set encoders for slideMotor
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set direction for drive motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Drive control using gamepad1
        double turn = gamepad1.left_stick_x; // forward/backward movement
        double drive = -gamepad1.right_stick_y; // turn left/right movement

        // Calculate power for drive motors
        double frontLeftPower = drive + turn;
        double frontRightPower = drive - turn;
        double backLeftPower = drive + turn;
        double backRightPower = drive - turn;

        // Set motor powers for driving
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // Control for slide motor using gamepad2 right stick
        double slidePower = -gamepad2.right_stick_y * armPowerScale; // Scale the power

        if (Math.abs(slidePower) > 0.1) { // Threshold to prevent accidental movement
            slideMotor.setPower(slidePower);

            // Update the target position to the current position for PID
            targetPosition = slideMotor.getCurrentPosition();

            // Disable PID control by resetting integral and error terms
            integral = 0;
            lastError = 0;
        } else {
            // Use PID control for slideMotor when joystick is not active
            double currentPosition = slideMotor.getCurrentPosition();
            double power = calculatePID(currentPosition, targetPosition, kP, kI, kD, integral, lastError);
            slideMotor.setPower(power);
        }

        // Display telemetry data
        telemetry.addData("Motors", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("SlideMotor Target", targetPosition);
        telemetry.addData("SlideMotor Current", slideMotor.getCurrentPosition());
        telemetry.update();
    }

    // PID calculation method
    private double calculatePID(double currentPosition, double targetPosition, double kP, double kI, double kD, double integral, double lastError) {
        double error = targetPosition - currentPosition; // Position error
        integral += error;                              // Accumulate error
        double derivative = error - lastError;          // Change in error
        lastError = error;                              // Update last error

        // PID formula
        double output = (kP * error) + (kI * integral) + (kD * derivative);

        // Clamp output to motor power range
        return Math.max(-1.0, Math.min(1.0, output));
    }
}