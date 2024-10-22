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
public class Claw extends LinearOpMode {

    DcMotorEx armLeft;
    DcMotorEx armRight;
    Servo claw;
    Servo joint;
    double clawOpen = 0.3, clawClose = 0.55;

    boolean isIntaking = false;
    boolean previousAState = false;
    public static double speedDivider = 2;
    public static double joystickSensitivity = 0.5;
    public static double jointPos = 0;


    // PID coefficients
    public static double Kp = 0.005;   // Proportional Gain
    public static double Ki = 0.01;    // Integral Gain
    public static double Kd = 0.001;    // Derivative Gain

    // Target position for the motor
    public static int targetPosition = 0;

    public static int maxPosition = 750;
    public static int minPosition = 0;

    // Integral and previous error for PID calculation
    private double integralSum = 0;
    private double lastError = 0;

    // Define a threshold for when to reset the integral sum
    public static final int POSITION_THRESHOLD = 5;

    // Time tracking for PID calculation
    private long lastTime;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class, "claw");
        joint = hardwareMap.get(Servo.class, "joint");
        armLeft = hardwareMap.get(DcMotorEx.class, "armLeft");
        armRight = hardwareMap.get(DcMotorEx.class, "armRight");
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setTargetPosition(targetPosition);
        armLeft.setTargetPosition(targetPosition);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        armLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        // FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();


        waitForStart();
        targetPosition = 0;

        lastTime = System.currentTimeMillis();

        while(opModeIsActive()){

            if (gamepad1.b)
                joint.setPosition(jointPos);

            if (gamepad1.x)
                claw.setPosition(0);


            // Read joystick input (assuming right stick Y-axis controls the motor position)
            double joystickInput = -gamepad1.right_stick_y;  // Negative because up is -1

            // Map the joystick input to a target position, move only when joystick is not zero
            if (Math.abs(joystickInput) > 0.05) {  // Small deadzone to prevent noise
                targetPosition += (int) (joystickInput * joystickSensitivity * maxPosition);

                // Clamp targetPosition to be within 0 and and the max position of the motor
                targetPosition = Math.max(0, Math.min(targetPosition, maxPosition));
            }
            if(gamepad1.dpad_right){ //init position of arm & joint
                targetPosition = (0);
                joint.setPosition(0);
                claw.setPosition(clawClose);
            }
            if(gamepad1.dpad_up){ // delivery position of the arm and joint

            }

            if(gamepad1.dpad_down){ // intake position of the arm and joint

            }

            //set to target position being calculated
            armLeft.setTargetPosition(targetPosition);
            armRight.setTargetPosition(targetPosition);

            // Get current motor position
            int currentPosition = armRight.getCurrentPosition();


            if (Math.abs(targetPosition - currentPosition) < POSITION_THRESHOLD) {
                integralSum = 0;  // Reset the integral sum
            }

            // Calculate error (difference between target and current position)
            double error = targetPosition - currentPosition;

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

            lastError = error;

            // Calculate final PID output
            double output = pTerm + iTerm + dTerm;

            // Limit motor power to [-1, 1]
            output = Math.max(-1, Math.min(output, 1));

            // Set motor power based on PID output
            armLeft.setPower(output/speedDivider);
            armRight.setPower(output/speedDivider);

            // Send telemetry to the dashboard
            telemetry.addData("Joystick Input", joystickInput);
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("PID Output", output);
            telemetry.update();



            // Toggle open/close claw when 'A' button is pressed
            boolean currentAState = gamepad1.a;

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


    /*
    joint intake pos = 0.78
    joint lift after intake pos = 0.9
    arm max pos = 870

     */
