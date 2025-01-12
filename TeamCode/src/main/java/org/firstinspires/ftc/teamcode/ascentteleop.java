package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class ascentteleop extends OpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor ascentMotor1;
    DcMotor ascentMotor2;

    // PID coefficients
    double kP1 = 0.01, kI1 = 0.0, kD1 = 0.0; // For ascentMotor1
    double kP2 = 0.01, kI2 = 0.0, kD2 = 0.0; // For ascentMotor2

    // Variables for PID
    double targetPosition1 = 0; // Target for ascentMotor1
    double targetPosition2 = 0; // Target for ascentMotor2

    double integral1 = 0, lastError1 = 0;
    double integral2 = 0, lastError2 = 0;

    double armPowerScale = 0.5;  // Set to 50% power; adjust as needed

    @Override
    public void init() {

        //initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");
        ascentMotor1 = hardwareMap.get(DcMotor.class, "arm1");
        ascentMotor2 = hardwareMap.get(DcMotor.class, "arm2");

        // Reset and set encoders
        ascentMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ascentMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        // Control for ascent motors using gamepad2 right stick
        double ascentPower = -gamepad2.right_stick_y * armPowerScale; // Scale the power

        if (Math.abs(ascentPower) > 0.1) { // Threshold to prevent accidental movement
            ascentMotor1.setPower(ascentPower);
            ascentMotor2.setPower(ascentPower);

            // Update the target positions to the current positions for PID
            targetPosition1 = ascentMotor1.getCurrentPosition();
            targetPosition2 = ascentMotor2.getCurrentPosition();

            // Disable PID control by resetting integral and error terms
            integral1 = 0;
            lastError1 = 0;
            integral2 = 0;
            lastError2 = 0;
        } else {
            // Use PID control for ascentMotor1 and ascentMotor2 when joystick is not active
            double currentPosition1 = ascentMotor1.getCurrentPosition();
            double power1 = calculatePID(currentPosition1, targetPosition1, kP1, kI1, kD1, integral1, lastError1);
            ascentMotor1.setPower(power1);

            double currentPosition2 = ascentMotor2.getCurrentPosition();
            double power2 = calculatePID(currentPosition2, targetPosition2, kP2, kI2, kD2, integral2, lastError2);
            ascentMotor2.setPower(power2);
        }

        //display telemetry data
        telemetry.addData("Motors", "FL: %.2f, FR: %.2f, BL:%.2f, BR: %.2f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("AscentMotor1 Target", targetPosition1);
        telemetry.addData("AscentMotor1 Current", ascentMotor1.getCurrentPosition());
        telemetry.addData("AscentMotor2 Target", targetPosition2);
        telemetry.addData("AscentMotor2 Current", ascentMotor2.getCurrentPosition());
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

