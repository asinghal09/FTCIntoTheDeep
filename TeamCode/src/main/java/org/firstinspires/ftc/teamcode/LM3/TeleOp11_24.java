package org.firstinspires.ftc.teamcode.LM3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@TeleOp

//joint pos for hanging specimens 0.6
public class TeleOp11_24 extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;

    DcMotorEx slidesJoint;
    DcMotorEx slides;
    Servo claw;
    //DigitalChannel touchSensor;

    private boolean moveForward = false;

    boolean isOpen = false;
    boolean previousAState = false;
    public static double clawOpen = 0.6;
    public static double clawClose = 0.3;

    public static double speedDivider = 2;

    //slide hard stops
    public static int maxSlidePos = 4400;
    public static int minSlidePos = 500;


    // PID coefficients
    public static double Kp = 0.0001;   // Proportional Gain
    public static double Ki = 0.00001;    // Integral Gain
    public static double Kd = 0.00001;    // Derivative Gain

    // Integral and previous error for PID calculation
    private double setpoint = 0;   // PID target position
    private double integralSum = 0;
    private double lastError = 0;
    private boolean pidEnabled = true;    //Track if PID is active


    // Time tracking for PID calculation
    private long lastTime;

    // Predefined positions for the arm
    public static int armInitPos = 200;
    public static int armIntakePos = 0;
    public static int armBasketPos = 2200;
    public static int armChamberPos = 1100;
    public static int maxSlideJointPos = 1650;
    public static int minJointPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "claw");

        slidesJoint = hardwareMap.get(DcMotorEx.class, "slidesJoint");
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        //touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        //touchSensor.setMode(DigitalChannel.Mode.INPUT);

        slidesJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesJoint.setDirection(DcMotorSimple.Direction.REVERSE);
        //slidesJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int slidesTargetPos = 0;
        slides.setTargetPosition(slidesTargetPos);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int slidesJointTargetPos = 0;
        slidesJoint.setTargetPosition(slidesJointTargetPos);
        slidesJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        if(isStopRequested()) return;

        lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {

        double driving = -gamepad1.right_stick_y*0.75; // Forward/backward
        double turning = gamepad1.left_stick_x * 0.5; // Turning
        double strafing = gamepad1.right_trigger - gamepad1.left_trigger; // Strafing

        // Combine inputs for each motor
        double frontLeftPower = driving + turning + strafing;
        double frontRightPower = driving - turning - strafing;
        double backLeftPower = driving + turning - strafing;
        double backRightPower = driving - turning + strafing;

        // Normalize powers to keep them within -1.0 to 1.0
        double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftPower /= maxPower;
        frontRightPower /= maxPower;
        backLeftPower /= maxPower;
        backRightPower /= maxPower;

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);


        if (gamepad2.left_bumper){
            frontLeft.setPower(-0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(-0.5);

            sleep(100);

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

        }

        if (gamepad2.right_bumper){
            slidesJointTargetPos = 1400;

        }

        if (gamepad2.y && !moveForward) {
            moveForward = true;
        }
        /*
        // If the robot is set to move forward
        if (moveForward) {
            frontLeft.setPower(0.25); // Move forward
            frontRight.setPower(0.25);
            backLeft.setPower(0.25);
            backRight.setPower(0.25);
            slidesTargetPos = 500;
            slidesJointTargetPos= 400;

            if (!touchSensor.getState()) { // Sensor is pressed (false when pressed)
                moveForward = false;
                frontLeft.setPower(0); // Stop the drive motors
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                slidesTargetPos = 750;

            }
        }

             */

            //slides controls
            slidesTargetPos += (int) (-gamepad2.left_stick_y * 30);

            if (slidesTargetPos < 0)
                slidesTargetPos = 0;
            slides.setTargetPosition(slidesTargetPos);
            slides.setPower(1);


            // Read joystick input, right stick Y-axis controls the motor position
            double joystickInput = -gamepad2.right_stick_y;

            //joint controls
            slidesJointTargetPos += (int) (joystickInput * 20);

            if (slidesJointTargetPos > maxSlideJointPos) {
                slidesJointTargetPos = maxSlideJointPos;
            } else if (slidesJointTargetPos <=0){
                slidesJointTargetPos = 0;
            }
            slidesJoint.setTargetPosition(slidesJointTargetPos);
            slidesJoint.setPower(0.5);

            if (gamepad2.left_trigger > 0.05){
                slidesJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slidesJointTargetPos = 0;
                slidesJoint.setTargetPosition(slidesJointTargetPos);
                slidesJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }


            // Check for button presses to set predefined positions
            if (gamepad2.dpad_down) {    //set arm and slides position for intaking
                setpoint = armIntakePos;
                slidesJointTargetPos = armIntakePos;
                slidesTargetPos = 50;
                pidEnabled = true;

            } else if (gamepad2.dpad_up) {    //arm and slide position for high basket
                setpoint = armBasketPos;
                slidesJointTargetPos = armBasketPos;
                pidEnabled = true;
                slidesTargetPos = 5000;

            } else if (gamepad2.dpad_left) {   //arm and slide pos for high chamber
                setpoint = armChamberPos;
                slidesJointTargetPos = armChamberPos;
                slidesTargetPos = 1050;
                pidEnabled = true;
            } else if (gamepad2.dpad_right) {   //arm and slide pos for high chamber
                setpoint = armInitPos;
                slidesJointTargetPos = armInitPos;
                pidEnabled = true;
            }


            if (slidesJointTargetPos > 0 && slidesJointTargetPos <850) {
                if (slidesTargetPos > 3365) {
                    slidesTargetPos = 3365;
                }
            }

            if (gamepad2.b){
                slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slidesTargetPos = 0;
                slides.setTargetPosition(slidesTargetPos);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }




            // Check if joystick is being moved
            /*if (Math.abs(joystickInput) > 0.05) {  // Threshold to avoid noise
                // Disable PID control and allow manual control
                pidEnabled = false;
                slidesJoint.setPower(joystickInput / speedDivider);

            } else if (!pidEnabled) {
                //joystick has been released
                setpoint = slidesJoint.getCurrentPosition();
                pidEnabled = true;
                integralSum = 0;
                lastError = 0;
            }
            // When joystick is not moving and PID is enabled, continue with PID control
            if (pidEnabled) {
                // Apply PID control
                double currentPosition = slidesJoint.getCurrentPosition();
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
                slidesJoint.setPower(output / speedDivider);

                lastError = error;
                telemetry.addData("output", output);
            }

             */
            // Send telemetry to the dashboard
            telemetry.addData("Joystick Input", joystickInput);
            telemetry.addData("PID Enabled", pidEnabled);
            //telemetry.addData("Setpoint", setpoint);
            telemetry.addData("Joint Setpoint", slidesJointTargetPos);
            telemetry.addData("Slides Joint Pos ", slidesJoint.getCurrentPosition());
            telemetry.addData("Slides Pos", slides.getCurrentPosition());
            telemetry.addData("Slides Target Position", slidesTargetPos);
            telemetry.update();


            // Toggle claw opening/closing when 'A' button is pressed
            boolean currentAState = gamepad2.a;
            if (currentAState && !previousAState) {
                // Toggle the open/close state
                isOpen = !isOpen;

                if (isOpen) {
                    claw.setPosition(clawOpen);
                } else {
                    claw.setPosition(clawClose);
                }
            }
            // Update the previous state of the 'A' button
            previousAState = currentAState;
        }
    }
}