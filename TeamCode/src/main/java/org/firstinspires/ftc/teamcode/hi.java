package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous

public class hi extends LinearOpMode {
    private DcMotor armMotor;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    //private PIDController armPIDController;

    // PID Constants
    private static final double KP_ARM = 0.1;
    private static final double KI_ARM = 0.0;
    private static final double KD_ARM = 0.05;

    // Target positions (in encoder ticks)
    private static final int TARGET_ARM_POSITION = 500; // Change this to your target position
    private static final double DRIVE_SPEED = 0.5;

    @Override
    public void runOpMode() {
        // Initialize motors
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Reset arm motor encoders
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initialize PID Controller
        //armPIDController = new PIDController(KP_ARM, KI_ARM, KD_ARM);

        waitForStart();

        // Move the arm to the target position using PID
        moveArmToTarget();

        // Drive forward for 2 seconds (example)
        driveForward(2.0);
    }

    private void moveArmToTarget() {
        armMotor.setTargetPosition(TARGET_ARM_POSITION);
        armMotor.setPower(1.0); // Start moving the arm

        // PID control loop for the arm
        while (opModeIsActive() && armMotor.isBusy()) {
            double currentPosition = armMotor.getCurrentPosition();
            //double output = armPIDController.calculate(TARGET_ARM_POSITION, currentPosition);

            //armMotor.setPower(output);

            telemetry.addData("Current Position", currentPosition);
            //telemetry.addData("PID Output", output);
            telemetry.update();
        }
        armMotor.setPower(0); // Stop the arm when done
    }

    private void driveForward(double seconds) {
        // Set motor modes for driving
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Drive forward
        frontLeft.setPower(DRIVE_SPEED);
        frontRight.setPower(DRIVE_SPEED);
        backLeft.setPower(DRIVE_SPEED);
        backRight.setPower(DRIVE_SPEED);

        sleep((long) (seconds * 1000));

        // Stop motors
        stopDrivetrain();
    }

    private void stopDrivetrain() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}