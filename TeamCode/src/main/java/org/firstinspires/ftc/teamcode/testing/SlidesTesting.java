package org.firstinspires.ftc.teamcode.testing;

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

public class SlidesTesting extends LinearOpMode {

    DcMotor slides;
    DcMotorEx slidesJoint;



    boolean isIntaking = false;
    boolean previousAState = false;

    public static double speedDividerSlides = 2;


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
    public static int armIntakePos = 735;
    public static int armDeliverPos = 460;
    public static int armDriveAroundPos = 170;

    @Override
    public void runOpMode() throws InterruptedException {

        slides = hardwareMap.get(DcMotorEx.class, "slides");
        slidesJoint = hardwareMap.get(DcMotorEx.class, "slidesJoint");


        //slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();



        waitForStart();

        lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {



            slides.setPower(gamepad1.left_stick_x/speedDividerSlides);


            // Read joystick input (assuming right stick Y-axis controls the motor position)
            double joystickInputRight = gamepad1.right_stick_y;

            // Check for button presses to set predefined positions
            if (gamepad2.dpad_right) {       //set arm & joint to init position

                setpoint = armInitPos;
                pidEnabled = true;

            } else if (gamepad2.dpad_down) {    //set arm and joint position for intaking
                //joint.setPosition();
                setpoint = armIntakePos;

                pidEnabled = true;
            } else if (gamepad2.dpad_up) {    //arm and joint position for delivery
                setpoint = armDeliverPos;

                pidEnabled = true;
            } else if (gamepad2.dpad_left){   //driving around position
                setpoint = armDriveAroundPos;

                pidEnabled = true;
            }

            // Check if joystick is being moved
            if (Math.abs(joystickInputRight) > 0.05) {  // Threshold to avoid noise
                // Disable PID control and allow manual control
                pidEnabled = false;
                slidesJoint.setPower(joystickInputRight / speedDivider);


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
            telemetry.addData("Joystick Input", joystickInputRight);
            telemetry.addData("PID Enabled", pidEnabled);
            telemetry.addData("Setpoint", setpoint);
            telemetry.addData("slides position", slides.getCurrentPosition());
            telemetry.addData("Joint Motor Position ", slidesJoint.getCurrentPosition());
            telemetry.update();



        }
    }
}
