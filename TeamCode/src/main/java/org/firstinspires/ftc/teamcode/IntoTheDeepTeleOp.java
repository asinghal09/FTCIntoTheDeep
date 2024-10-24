package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp

public class IntoTheDeepTeleOp extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;

    DcMotorEx armLeft;
    DcMotorEx armRight;
    Servo claw;
    Servo joint;


    boolean isIntaking = false;
    boolean previousAState = false;
    public static double clawOpen = 0.225;
    public static double clawClose = 0.4;
    public static double speedDivider = 8;
    public static double jointPos = 0.3;


    // PID coefficients
    public static double Kp = 0.019;   // Proportional Gain
    public static double Ki = 0.00015;    // Integral Gain
    public static double Kd = 0;    // Derivative Gain

    // Integral and previous error for PID calculation
    private double setpoint = 0;   // PID target position
    private double integralSum = 0;
    private double lastError = 0;
    private boolean pidEnabled = true;    //Track if PID is active


    // Time tracking for PID calculation
    private long lastTime;

    // Predefined positions for the arm
    public static int armInitPos = 0;
    public static int armIntakePos = 740;
    public static int armDeliverPos = 490;
    public static int armDriveAroundPos = 170;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        claw = hardwareMap.get(Servo.class, "claw");
        joint = hardwareMap.get(Servo.class, "joint");
        armLeft = hardwareMap.get(DcMotorEx.class, "armLeft");
        armRight = hardwareMap.get(DcMotorEx.class, "armRight");

        armLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        joint.setPosition(0.1);

        waitForStart();
        lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {

            double driveLeft = gamepad1.right_stick_y*-0.75;
            double driveRight = gamepad1.left_stick_y*-0.75;
            double strafe = gamepad1.right_stick_x/2;

            frontLeft.setPower(driveLeft);
            frontRight.setPower(driveRight);
            backLeft.setPower(driveLeft);
            backRight.setPower(driveRight);
            if (gamepad1.left_bumper){
                frontLeft.setPower(-0.5);
                frontRight.setPower(0.5);
                backLeft.setPower(0.5);
                backRight.setPower(-0.5);
            }
            if (gamepad1.right_bumper){
                frontLeft.setPower(0.5);
                frontRight.setPower(-0.5);
                backLeft.setPower(-0.5);
                backRight.setPower(0.5);
            }


            if (gamepad2.b){
                joint.setPosition(jointPos);}
            else if (gamepad2.left_bumper){
                joint.setPosition(joint.getPosition()+0.05);
            } else if (gamepad2.right_bumper){
                joint.setPosition(joint.getPosition()-0.05);
            }


            // Read joystick input (assuming right stick Y-axis controls the motor position)
            double joystickInput = gamepad2.right_stick_y;

            // Check for button presses to set predefined positions
            if (gamepad2.dpad_right) {       //set arm & joint to init position
                joint.setPosition(0.075);
                claw.setPosition(clawClose);
                setpoint = armInitPos;
                pidEnabled = true;

            } else if (gamepad2.dpad_down) {    //set arm and joint position for intaking
                //joint.setPosition();
                setpoint = armIntakePos;
                joint.setPosition(0.15);
                pidEnabled = true;
            } else if (gamepad2.dpad_up) {    //arm and joint position for delivery
                setpoint = armDeliverPos;
                joint.setPosition(0.4);
                pidEnabled = true;
            } else if (gamepad2.dpad_left){   //driving around position
                setpoint = armDriveAroundPos;
                joint.setPosition(0.8);
                pidEnabled = true;
            }

            // Check if joystick is being moved
            if (Math.abs(joystickInput) > 0.05) {  // Threshold to avoid noise
                // Disable PID control and allow manual control
                pidEnabled = false;
                armLeft.setPower(joystickInput / speedDivider);
                armRight.setPower(joystickInput / speedDivider);

            } else if (!pidEnabled) {
                //joystick has been released
                setpoint = armRight.getCurrentPosition();
                pidEnabled = true;
                integralSum = 0;
                lastError = 0;
            }
            // When joystick is not moving and PID is enabled, continue with PID control
            if (pidEnabled) {
                // Apply PID control
                double currentPosition = armRight.getCurrentPosition();
                double error = setpoint - currentPosition;


                // Calculate time step (delta time)
                long currentTime = System.currentTimeMillis();
                double deltaTime = (currentTime - lastTime) / 1000.0;  // Convert to seconds
                lastTime = currentTime;


                // Proportional term
                double pTerm = Kp * error;

                // Integral term
                integralSum += error * deltaTime;
                double iTerm = Ki * integralSum;

                // Derivative term
                double derivative = (error - lastError) / deltaTime;
                double dTerm = Kd * derivative;

                // Calculate final PID output
                double output = pTerm + iTerm + dTerm;

                // Limit motor power to [-1, 1]
                output = Math.max(-1, Math.min(output, 1));

                // Set motor power based on PID output
                armLeft.setPower(output / speedDivider);
                armRight.setPower(output / speedDivider);

                lastError = error;
                telemetry.addData("output", output);
            }
            // Send telemetry to the dashboard
            telemetry.addData("Joystick Input", joystickInput);
            telemetry.addData("PID Enabled", pidEnabled);
            telemetry.addData("Setpoint", setpoint);

            telemetry.addData("Motor Position ", armRight.getCurrentPosition());
            telemetry.update();


            // Toggle open/close claw when 'A' button is pressed
            boolean currentAState = gamepad2.a;

            if (currentAState && !previousAState) {
                // Toggle the spinning state
                isIntaking = !isIntaking;

                // If spinning, set servo speed
                if (isIntaking) {
                    claw.setPosition(clawClose);
                } else {       // If not spinning, stop the servo
                    claw.setPosition(clawOpen);
                }
            }
            // Update the previous state of the 'A' button
            previousAState = currentAState;
        }
    }
}
