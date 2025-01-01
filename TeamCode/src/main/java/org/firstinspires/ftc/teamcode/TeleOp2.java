package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class TeleOp2 extends OpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor joint1Motor;
    DcMotor joint2Motor;
    Servo endJointServo;
    Servo clawServo;

    // PID coefficients
    double kP1 = 0.01, kI1 = 0.0, kD1 = 0.0; // For joint1Motor
    double kP2 = 0.01, kI2 = 0.0, kD2 = 0.0; // For joint2Motor

    // Variables for PID
    double targetPosition1 = 0; // Target for joint1Motor
    double targetPosition2 = 0; // Target for joint2Motor

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
        joint1Motor = hardwareMap.get(DcMotor.class, "arm1");
        joint2Motor = hardwareMap.get(DcMotor.class, "arm2");


        // Reset and set encoders
        joint1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint1Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        joint2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set direction
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Initialize servos

        endJointServo = hardwareMap.get(Servo.class, "joint");
        clawServo = hardwareMap.get(Servo.class, "claw");

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


        // Manual control for joint1Motor using gamepad2 left stick
        double manualPower1 = -gamepad2.left_stick_y; // Invert for intuitive control

        if (Math.abs(manualPower1) > 0.1) { // Threshold to prevent accidental movement
            joint1Motor.setPower(manualPower1);

            // Update the target position to the current position for PID
            targetPosition1 = joint1Motor.getCurrentPosition();

            // Disable PID control by resetting integral and error terms
            integral1 = 0;
            lastError1 = 0;
        } else {
            // Use PID control for joint1Motor when joystick is not active
            double currentPosition1 = joint1Motor.getCurrentPosition();
            double power1 = calculatePID(currentPosition1, targetPosition1, kP1, kI1, kD1, integral1, lastError1);
            joint1Motor.setPower(power1);
        }

        // Manual control for joint2Motor using gamepad2 right stick
        double manualPower2 = -gamepad2.right_stick_y; // Invert for intuitive control

        if (Math.abs(manualPower2) > 0.1) { // Threshold to prevent accidental movement
            joint2Motor.setPower(manualPower2);

            // Update the target position to the current position for PID
            targetPosition2 = joint2Motor.getCurrentPosition();

            // Disable PID control by resetting integral and error terms
            integral2 = 0;
            lastError2 = 0;
        } else {
            // Use PID control for joint2Motor when joystick is not active
            double currentPosition2 = joint2Motor.getCurrentPosition();
            double power2 = calculatePID(currentPosition2, targetPosition2, kP2, kI2, kD2, integral2, lastError2);
            joint2Motor.setPower(power2);
        }


        // Button triggers for preset positions
        if (gamepad2.a) { //for specimen
            // Preset 1: Picking position
            targetPosition1 = 1300;  // Joint 1 target in encoder ticks
            targetPosition2 = 900;  // Joint 2 target in encoder ticks
            endJointServo.setPosition(0.5); // end joint position
            clawServo.setPosition(0.5);    // Claw open
        } else if (gamepad2.y) { //for basket
            // Preset 2: Placing position
            targetPosition1 = 1400;
            targetPosition2 = 700;
            endJointServo.setPosition(0.5);
            clawServo.setPosition(0.5);    // Claw mid-open
        } else if (gamepad2.x) {
            // Preset 3: Rest position
            targetPosition1 = 0;
            targetPosition2 = 0;
            endJointServo.setPosition(0.0);
            clawServo.setPosition(0.0);    // Claw closed
        }

        // PID control for joint1Motor
        double currentPosition1 = joint1Motor.getCurrentPosition();
        double power1 = calculatePID(currentPosition1, targetPosition1, kP1, kI1, kD1, integral1, lastError1) * armPowerScale;
        joint1Motor.setPower(power1);

        // PID control for joint2Motor
        double currentPosition2 = joint2Motor.getCurrentPosition();
        double power2 = calculatePID(currentPosition2, targetPosition2, kP2, kI2, kD2, integral2, lastError2) * armPowerScale;
        joint2Motor.setPower(power2);


        //display telemetry data
        telemetry.addData("Motors", "FL: %.2f, FR: %.2f, BL:%.2f, BR: %.2f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("Joint1 Target", targetPosition1);
        telemetry.addData("Joint1 Current", currentPosition1);
        telemetry.addData("Joint1 Power", power1);
        telemetry.addData("Joint2 Target", targetPosition2);
        telemetry.addData("Joint2 Current", currentPosition2);
        telemetry.addData("Joint2 Power", power2);
        telemetry.addData("Joint Servo", endJointServo.getPosition());
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
