package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class ascentslides extends OpMode{
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor slideMotor;  // Single motor for slide
    Servo clawServo;

    // PID coefficients
    double kP = 0.01, kI = 0.0, kD = 0.0; // For slideMotor

    // Variables for PID
    double targetPosition = 0; // Target for slideMotor

    double integral = 0, lastError = 0;

    boolean clawOpen = false; // Tracks claw state
    boolean clawTogglePressed = false; // Tracks button press for toggling

    double armPowerScale = 0.5;  // Set to 50% power; adjust as needed


    @Override
    public void init() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        clawServo = hardwareMap.get(Servo.class, "claw");

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
        double turn = gamepad1.left_stick_x; //forward and backward
        double drive = -gamepad1.right_stick_y; //turn right and left
        double strafe = gamepad1.right_trigger - gamepad1.left_trigger;

        //calculate power
        double frontLeftPower = drive + turn + strafe;
        double frontRightPower = drive - turn - strafe; 0: frontLeft 1: backleft 2: frontright 3: backright
        double backLeftPower = drive + turn - strafe;
        double backRightPower = drive - turn + strafe;

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
        if (gamepad2.b && !clawTogglePressed) {
            clawOpen = !clawOpen; // Toggle claw state
            clawTogglePressed = true; // Prevent multiple toggles on one press
            if (clawOpen) {
                clawServo.setPosition(1.0); // Fully open claw
            } else {
                clawServo.setPosition(0.0); // Fully close claw
            }
        } else if (!gamepad2.b) {
            clawTogglePressed = false; // Reset toggle state
        }

        // Display telemetry data
        telemetry.addData("Motors", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("SlideMotor Target", targetPosition);
        telemetry.addData("SlideMotor Current", slideMotor.getCurrentPosition());
        telemetry.addData("Claw Servo", clawServo.getPosition());
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