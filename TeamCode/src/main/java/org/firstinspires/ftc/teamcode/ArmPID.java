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
public class ArmPID extends LinearOpMode {

    DcMotorEx armLeft;
    DcMotorEx armRight;
    Servo claw;
    Servo joint;


    boolean isIntaking = false;
    boolean previousAState = false;
    double clawOpen = 0.3, clawClose = 0.55;
    public static double speedDivider = 4;
    public static double jointPos = 0;


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
     int armInitPos= 100;
     int armIntakePos = 200;
     int armDeliverPos = 300;
    @Override
    public void runOpMode() throws InterruptedException {
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


        waitForStart();
        lastTime = System.currentTimeMillis();

        while(opModeIsActive()){

            if (gamepad1.b)
                joint.setPosition(jointPos);

            if (gamepad1.x)
                claw.setPosition(0);


            // Read joystick input (assuming right stick Y-axis controls the motor position)
            double joystickInput = -gamepad1.right_stick_y;  // Negative because up is -1

            // Check for button presses to set predefined positions
            if (gamepad1.dpad_right) {       //set arm & joint to init position
                joint.setPosition(0);
                claw.setPosition(clawClose);
                setpoint = armInitPos;
                pidEnabled = true;
            } else if (gamepad1.dpad_down) {    //set arm and joint position for intaking
                //joint.setPosition();
                setpoint = armIntakePos;
                pidEnabled = true;
            } else if (gamepad1.dpad_up) {

                setpoint = armDeliverPos;
                pidEnabled = true;
            }

            // Check if joystick is being moved
            if (Math.abs(joystickInput) > 0.05) {  // Threshold to avoid noise
                // Disable PID control and allow manual control
                pidEnabled = false;
                armLeft.setPower(joystickInput/speedDivider);
                armRight.setPower(joystickInput/speedDivider);
                
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
                double derivative = (error - lastError)/deltaTime;
                double dTerm = Kd * derivative;

                // Calculate final PID output
                double output = pTerm + iTerm + dTerm;

                // Limit motor power to [-1, 1]
                output = Math.max(-1, Math.min(output, 1));

                // Set motor power based on PID output
                armLeft.setPower(output /speedDivider);
                armRight.setPower(output /speedDivider);

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
        }   boolean currentAState = gamepad1.a;

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



    /*
    joint intake pos = 0.78
    joint lift after intake pos = 0.9
    arm max pos = 870

     */

