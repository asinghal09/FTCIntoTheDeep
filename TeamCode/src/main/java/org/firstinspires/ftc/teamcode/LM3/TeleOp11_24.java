package org.firstinspires.ftc.teamcode.LM3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


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
    public static int armInitPos = 2;
    public static int armIntakePos = 675;
    public static int armBasketPos = 2200;
    public static int armChamberPos = 800;
    public static int maxSlideJointPos = 1400;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        claw = hardwareMap.get(Servo.class, "claw");

        slidesJoint = hardwareMap.get(DcMotorEx.class, "slidesJoint");
        slides = hardwareMap.get(DcMotorEx.class, "slides");

        slidesJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesJoint.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

            //strafing controls
            if (gamepad1.left_trigger > 0){
                frontLeft.setPower(gamepad1.left_trigger);
                frontRight.setPower(-gamepad1.left_trigger);
                backLeft.setPower(-gamepad1.left_trigger);
                backRight.setPower(gamepad1.left_trigger);
            }
            if (gamepad1.right_trigger > 0){
                frontLeft.setPower(-gamepad1.right_trigger);
                frontRight.setPower(gamepad1.right_trigger);
                backLeft.setPower(gamepad1.right_trigger);
                backRight.setPower(-gamepad1.right_trigger);
            }


            //slides controls
            slidesTargetPos += (int)(-gamepad2.left_stick_y * 30);

            if (slidesTargetPos > maxSlidePos) {
                slidesTargetPos = maxSlidePos;
            }
            slides.setTargetPosition(slidesTargetPos);
            slides.setPower(0.5);

           /*
            if (gamepad1.dpad_up)
                slidesTargetPos += 50;

            if (gamepad1.dpad_down)
                slidesTargetPos -= 50;

            if (slidesTargetPos > maxSlidePos)
                slidesTargetPos = maxSlidePos;
            else if (slidesTargetPos < 0)
                slidesTargetPos = 0;
            slides.setTargetPosition(slidesTargetPos);
            slides.setPower(0.5);

            //movement of arm
            if (gamepad1.dpad_left)
                slidesJointTargetPos += 20;
            else if (gamepad1.dpad_right)
                slidesJointTargetPos -= 20;
            if (slidesJointTargetPos < 0)
                slidesJointTargetPos = 0;
            slidesJoint.setTargetPosition(slidesJointTargetPos);
            slidesJoint.setPower(0.5);


            telemetry.addData("slides target pos ", slidesTargetPos);
            telemetry.addData("slides actual pos", slides.getCurrentPosition());
            telemetry.addData("joint targetpos ", slidesJointTargetPos);
            telemetry.addData("joint actual pos", slidesJoint.getCurrentPosition());
            telemetry.update();
            */



            // Read joystick input, right stick Y-axis controls the motor position
            double joystickInput = -gamepad2.right_stick_y;

            // Check for button presses to set predefined positions
            if (gamepad2.dpad_down) {    //set arm and slides position for intaking
                setpoint = armInitPos;
                slidesTargetPos = 0;
                pidEnabled = true;

            } else if (gamepad2.dpad_up) {    //arm and slide position for high basket
                setpoint = armBasketPos;
                pidEnabled = true;
                slidesTargetPos = 5000;

            } else if (gamepad2.dpad_left){   //arm and slide pos for high chamber
                setpoint = armChamberPos;
                slidesTargetPos = 500;
                pidEnabled = true;
            }

            if(slidesJoint.getCurrentPosition() > maxSlideJointPos){
                pidEnabled = true;
                setpoint = maxSlideJointPos;
            }


            // Check if joystick is being moved
            if (Math.abs(joystickInput) > 0.05) {  // Threshold to avoid noise
                // Disable PID control and allow manual control
                pidEnabled = false;
                slidesJoint.setPower(joystickInput / speedDivider);

            }else if (!pidEnabled) {
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