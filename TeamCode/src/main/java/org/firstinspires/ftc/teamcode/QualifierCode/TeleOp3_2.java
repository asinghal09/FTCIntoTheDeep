package org.firstinspires.ftc.teamcode.QualifierCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@TeleOp

public class TeleOp3_2 extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;

    DcMotorEx slidesJoint;
    DcMotorEx slides;
    Servo claw, joint, spinny;

    ElapsedTime timer;
    //DigitalChannel touchSensor;

    private boolean moveForward = false;

    boolean isOpen = false;
    boolean previousAState = false;


    boolean isUp = false;               //for toggling joint up vs down position based on left bumper
    boolean previousLBState = false;
    public static double clawOpen = 0.6;
    public static double clawClose = 0.35;

    public static double spinnyNormalPos = 0.61;

    public static double spinnyPos = 0.5;
    public static double jointServoPos = 0.5;

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
    public static int armInitPos = 0;
    public static int armDrivingAroundPos = 450;
    public static int armBasketPos = 1650;
    public static int armChamberPos = 1300;
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
        joint = hardwareMap.get(Servo.class, "joint");
        spinny = hardwareMap.get(Servo.class, "spinny");

        slidesJoint = hardwareMap.get(DcMotorEx.class, "slidesJoint");
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        //touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        //touchSensor.setMode(DigitalChannel.Mode.INPUT);

        timer = new ElapsedTime();

        slidesJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesJoint.setDirection(DcMotorSimple.Direction.REVERSE);
        //slidesJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int slidesTargetPos = 0;
        slides.setTargetPosition(slidesTargetPos);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(1);
        int slidesJointTargetPos = 0;
        slidesJoint.setTargetPosition(slidesJointTargetPos);
        slidesJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();
        timer.reset();

        if(isStopRequested()) return;

        lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {

        if(timer.seconds() > 15){
            if (slidesJointTargetPos > maxSlideJointPos) {          //joint hardstops
                slidesJointTargetPos = maxSlideJointPos;
            } else
            if (slidesJointTargetPos <0){
                slidesJointTargetPos = 0;
            }
            slidesJoint.setTargetPosition(slidesJointTargetPos);
            slidesJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (slidesTargetPos < 0) //slides hardstops
                slidesTargetPos = 0;
            else if(slidesTargetPos>maxSlidePos)
                slidesTargetPos = maxSlidePos;

            slides.setTargetPosition(slidesTargetPos);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slides.setPower(1);
        }

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

        if (gamepad2.right_bumper)
            slidesJointTargetPos = 900;

        //slides controls
        slidesTargetPos += (int) (-gamepad2.left_stick_y * 30);

        slides.setTargetPosition(slidesTargetPos);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(1);

        // Read joystick input, right stick Y-axis controls the motor position
        double joystickInput = -gamepad2.right_stick_y;

            //joint controls
            slidesJointTargetPos += (int) (joystickInput * 20);

            slidesJoint.setTargetPosition(slidesJointTargetPos);
            slidesJoint.setPower(0.5);

            //code for manual reset of slides and slidesjoint hardstop
            //NEEDS TO BE RE-ADDED AFTER DONE WITH TESTING

            if (gamepad2.back){
                slidesJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slidesJointTargetPos = 0;
                slidesJoint.setTargetPosition(slidesJointTargetPos);
                slidesJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slidesTargetPos = 0;
                slides.setTargetPosition(slidesTargetPos);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // Check for button presses to set predefined positions
            if (gamepad2.dpad_down) {    //set arm and slides position for intaking/init
                setpoint = armInitPos;
                slidesJointTargetPos = armInitPos;
                slidesTargetPos = 0;
                pidEnabled = true;
                jointServoPos = 0.5;
                spinnyPos = spinnyNormalPos;

            } else if (gamepad2.dpad_up) {    //arm and slide position for high basket
                setpoint = armBasketPos;
                slidesJointTargetPos = armBasketPos;
                pidEnabled = true;
                slidesTargetPos = 3950;
                jointServoPos = 0.5;
                spinnyPos = spinnyNormalPos;

            } else if (gamepad2.dpad_left) {   //arm and slide pos for high chamber
                setpoint = armChamberPos;
                slidesJointTargetPos = armChamberPos;
                slidesTargetPos = 675;
                pidEnabled = true;
                spinnyPos = spinnyNormalPos;
            } else if (gamepad2.dpad_right) {   //arm and slide pos for Sub intaking
                setpoint = armDrivingAroundPos;
                slidesJointTargetPos = armDrivingAroundPos;
                pidEnabled = true;
                jointServoPos = 0;
                spinnyPos = spinnyNormalPos;
            }

            if (slidesJointTargetPos > 0 && slidesJointTargetPos <850) {
                if (slidesTargetPos > 3000) {
                    slidesTargetPos = 3000;
                }
            }

            if (gamepad2.x){
                jointServoPos += 0.05;
            }
            else if (gamepad2.y){
                jointServoPos -= 0.05;
            }
            joint.setPosition(jointServoPos);

            spinnyPos += (gamepad2.left_trigger/15) - (gamepad2.right_trigger/15);
            if (spinnyPos > 1)
                    spinnyPos = 1;
            else if (spinnyPos < 0)
                spinnyPos = 0;

            if(gamepad2.b){
                spinnyPos = spinnyNormalPos;
            }

            spinny.setPosition(spinnyPos);

            // Send telemetry to the dashboard
            telemetry.addData("Joystick Input", joystickInput);
            //telemetry.addData("PID Enabled", pidEnabled);
            //telemetry.addData("Setpoint", setpoint);
            telemetry.addData("Joint Target", slidesJointTargetPos);
            telemetry.addData("Slides Joint Pos ", slidesJoint.getCurrentPosition());
            telemetry.addData("Slides Pos", slides.getCurrentPosition());
            telemetry.addData("Slides Target Position", slidesTargetPos);
            telemetry.addData("Spinny Target Pos",spinnyPos);
            telemetry.addData("Spinny Actual Pos", spinny.getPosition());
            telemetry.addData("Joint Target Pos",jointServoPos);
            telemetry.addData("Joint Actual Pos", joint.getPosition());
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

            boolean currentLBState = gamepad2.left_bumper;
            if (currentLBState && !previousLBState) {
                // Toggle the state of the claw's servo joint
                isUp = !isUp;

                if (isUp) {
                    jointServoPos = 0;
                } else {
                    jointServoPos = 0.5;
                }
            }
            // Update the previous state of the 'LB' button
            previousLBState = currentLBState;
        }
    }
}