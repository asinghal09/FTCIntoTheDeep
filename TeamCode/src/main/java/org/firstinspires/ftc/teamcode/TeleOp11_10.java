package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp

//joint pos for hanging specimens 0.6
public class TeleOp11_10 extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;

    DcMotorEx slidesJoint;
    DcMotorEx slides;
    CRServo spinnyWheels;
    Servo joint;


    boolean isIntaking = false;
    boolean previousAState = false;
    boolean isDelivering = false;
    boolean previousBState = false;

    public static double speedDivider = 2;
    public static double jointPosSpecimenPickUp = 0.5;
    public static double jointBucketMore = 0.6;

    //slide hard stops
    public static int maxSlidePos = 5000;
    public static int minSlidePos = 500;

    // PID coefficients
    public static double Kp = 0.019;   // Proportional Gain
    public static double Ki = 0.00015;    // Integral Gain
    public static double Kd = 0.00001;    // Derivative Gain

    // Integral and previous error for PID calculation
    private double setpoint = 0;   // PID target position
    private double integralSum = 0;
    private double lastError = 0;
    private boolean pidEnabled = true;    //Track if PID is active


    // Time tracking for PID calculation
    private long lastTime;

    // Predefined positions for the arm
    public static int armInitPos = 2;
    public static int armIntakePos = 675;
    public static int armBasketPos = 2200;
    public static int armChamberPos = 1670;
    public static int maxSlideJointPos = 2400;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        spinnyWheels = hardwareMap.get(CRServo.class, "spinnyWheels");
        joint = hardwareMap.get(Servo.class, "joint");
        slidesJoint = hardwareMap.get(DcMotorEx.class, "slidesJoint");
        slides = hardwareMap.get(DcMotorEx.class, "slides");

        slidesJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesJoint.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        int slidesTargetPos = 0;
        slides.setTargetPosition(slidesTargetPos);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();
        if(isStopRequested()) return;

        lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {

            double driveLeft = gamepad1.left_stick_y*0.75;
            double driveRight = gamepad1.right_stick_y*0.75;
            //double strafe = gamepad1.right_stick_x/2;

            //driving controls
            frontLeft.setPower(driveLeft);
            frontRight.setPower(driveRight);
            backLeft.setPower(driveLeft);
            backRight.setPower(driveRight);
            if (gamepad1.left_bumper){
                frontLeft.setPower(0.5);
                frontRight.setPower(-0.5);
                backLeft.setPower(-0.5);
                backRight.setPower(0.5);
            }
            if (gamepad1.right_bumper){
                frontLeft.setPower(-0.5);
                frontRight.setPower(0.5);
                backLeft.setPower(0.5);
                backRight.setPower(-0.5);
            }

            if (gamepad2.x){
                joint.setPosition(jointPosSpecimenPickUp);}

            if (gamepad2.y){
                joint.setPosition(jointBucketMore);}

            slidesTargetPos += (int)(-gamepad2.left_stick_y * 30);

            if (slidesTargetPos > maxSlidePos) {
                slidesTargetPos = maxSlidePos;
            } else if (setpoint>0 && setpoint < 1000){
                if (slidesTargetPos > minSlidePos){
                    slidesTargetPos = minSlidePos;
                }
            }
            if (setpoint > 2350) {
                if (slidesTargetPos > 4000){
                    slidesTargetPos = 4000;
                }

            }
            slides.setTargetPosition(slidesTargetPos);
            slides.setPower(0.75);

            // Read joystick input (assuming right stick Y-axis controls the motor position)
            double joystickInput = -gamepad2.right_stick_y;

            // Check for button presses to set predefined positions
            if (gamepad2.dpad_right) {       //set arm & joint to init position
                joint.setPosition(0.125);
                setpoint = armInitPos;
                pidEnabled = true;
                slidesTargetPos = 0;
            } else if (gamepad2.dpad_down) {    //set arm and joint position for intaking
                setpoint = armIntakePos;
                joint.setPosition(0.73);
                slidesTargetPos = 0;
                pidEnabled = true;
            } else if (gamepad2.dpad_up) {    //arm and joint position for high basket
                setpoint = armBasketPos;
                joint.setPosition(0.7);
                pidEnabled = true;
                slidesTargetPos = 5000;
            } else if (gamepad2.dpad_left){   //arm and joint pos for high chamber
                setpoint = armChamberPos;
                slidesTargetPos = 500;
                joint.setPosition(0.625);
                pidEnabled = true;
            }

            if(slidesJoint.getCurrentPosition() > maxSlideJointPos){
                pidEnabled = true;
                setpoint = maxSlideJointPos;
            }

            if (gamepad2.left_trigger > 0){
                slidesJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setpoint = 0;
                slidesJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slidesTargetPos = 0;
                slides.setTargetPosition(slidesTargetPos);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.right_trigger > 0){
                joint.setPosition(0.3);
            }


            // Check if joystick is being moved
            if (Math.abs(joystickInput) > 0.05) {  // Threshold to avoid noise
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
            // Send telemetry to the dashboard
            telemetry.addData("Joystick Input", joystickInput);
            telemetry.addData("PID Enabled", pidEnabled);
            telemetry.addData("Setpoint", setpoint);
            telemetry.addData("Slides Joint Pos ", slidesJoint.getCurrentPosition());
            telemetry.addData("Slides Pos", slides.getCurrentPosition());
            telemetry.addData("Slides Target Position", slidesTargetPos);
            telemetry.addData("Joint Position", joint.getPosition());
            telemetry.update();

            // Toggle spinning wheels for DELIVERING when 'A' button is pressed
            boolean currentAState = gamepad2.a;
            if (currentAState && !previousAState) {
                // Toggle the spinning state
                isIntaking = !isIntaking;

                // If spinning, set servo speed
                if (isIntaking) {
                    spinnyWheels.setPower(0.2);
                } else {       // If not DELIVERING, stop the servo
                    spinnyWheels.setPower(0);
                }
            }
            // Update the previous state of the 'A' button
            previousAState = currentAState;

            //INTAKE
            boolean currentBState = gamepad2.b;
            if (currentBState && !previousBState) {
                // Toggle the spinning state
                isDelivering = !isDelivering;

                // If spinning, set servo speed
                if (isDelivering) {
                    spinnyWheels.setPower(-0.5);
                } else {       // If done INTAKING, stop the servo
                    spinnyWheels.setPower(0);
                }
            }
            // Update the previous state of the 'A' button
            previousBState = currentBState;
        }
    }
}