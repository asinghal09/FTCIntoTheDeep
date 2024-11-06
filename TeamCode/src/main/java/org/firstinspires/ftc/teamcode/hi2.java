package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous
public class hi2 extends LinearOpMode {

    // Declare motors and other hardware
    private DcMotorEx armMotor = null; // Arm motor
    private DcMotor leftFront, leftRear, rightFront, rightRear; // Drivetrain motors
    private ElapsedTime runtime = new ElapsedTime();

    // PID control variables for the arm
    private double kP = 0.01;  // Proportional gain
    private double kI = 0.0;   // Integral gain
    private double kD = 0.0;   // Derivative gain
    private double targetPosition = 500;  // Target encoder ticks for arm position
    private int armMotorPosition = 0;

    // Drivetrain control variables
    private double driveSpeed = 0.5;

    // Time duration for arm movement
    private double armMoveDuration = 3.0; // 3 seconds to move the arm

    @Override
    public void runOpMode() {
        // Initialize hardware
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Set motor directions
        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Set up PID for arm motor
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, 0);
        armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // Wait for the start button to be pressed
        waitForStart();

        // Reset the runtime timer
        runtime.reset();

        // Run the arm movement and drivetrain for 3 seconds
        while (opModeIsActive() && runtime.seconds() <= armMoveDuration) {
            // Get current encoder position of the arm motor
            armMotorPosition = armMotor.getCurrentPosition();

            // Determine the target position based on elapsed time
            targetPosition = 500;  // Set target position to move arm upwards

            // PID control for arm motor to reach the target position
            double error = targetPosition - armMotorPosition;
            double pidOutput = calculatePID(error);

            armMotor.setPower(pidOutput);

            // Simple drivetrain control - Move forward during this period
            leftFront.setPower(driveSpeed);
            leftRear.setPower(driveSpeed);
            rightFront.setPower(driveSpeed);
            rightRear.setPower(driveSpeed);

            // Display information for debugging
            telemetry.addData("Arm Position", armMotorPosition);
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("PID Output", pidOutput);
            telemetry.addData("Time Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Stop all motors once the 3 seconds have passed
        armMotor.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        // Display final telemetry data
        telemetry.addData("Final Arm Position", armMotor.getCurrentPosition());
        telemetry.addData("Mission Complete", "Stopping Motors");
        telemetry.update();
    }

    // PID calculation method
    private double calculatePID(double error) {
        double proportional = kP * error;
        double integral = kI * (runtime.time() * error);  // Simplified Integral (for simplicity)
        double derivative = kD * (error / runtime.time());
        return Range.clip(proportional + integral + derivative, -1.0, 1.0);  // Clip to [-1,1] for motor power
    }
}