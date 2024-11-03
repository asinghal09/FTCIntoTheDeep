package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/* Defines as an autonomous OpMode with name
OpmMode is something that runs under specific conditions
 */
@Autonomous(name = "TestAuto")
public class Test_Aniva extends LinearOpMode {

    // defines motors for each wheel
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    // Position counters to track target encoder positions
    private int backLeftPos;
    private int backRightPos;
    private int frontLeftPos;
    private int frontRightPos;

    @Override
    public void runOpMode() {
        // initialize motors by mapping them to names in the configuration file
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");

        // Reset encoders to zero for accurate tracking
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set directions for right-side motors to move in reverse, adjusting for physical orientation
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initialize encoder position counters to zero
        backLeftPos = 0;
        backRightPos = 0;
        frontRightPos = 0;
        frontLeftPos = 0;

        //Wait for start button to begin
        waitForStart();


        drive(1000,1000, 1000, 1000, 0.25);
        drive(1000,-1000, 1000, -1000, 0.25);
    }

    private void drive(double backLeftTarget, double backRightTarget, double frontLeftTarget, double frontRightTarget, double speed) {
        backLeftPos += backLeftTarget;
        backRightPos += backRightTarget;
        frontLeftPos += frontLeftTarget;
        frontRightPos += frontRightTarget;

        backLeft.setTargetPosition(backLeftPos);
        backRight.setTargetPosition(backRightPos);
        frontLeft.setTargetPosition(frontLeftPos);
        frontRight.setTargetPosition(frontRightPos);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLeft.setPower(speed);
        backRight.setPower(speed);
        frontLeft.setPower(speed);
        frontRight.setPower(speed);

        while (opModeIsActive() && backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy()) {
            idle();
        }
    }
}