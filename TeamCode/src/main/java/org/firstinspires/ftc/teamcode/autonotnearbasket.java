package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous
public class autonotnearbasket extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    int frontLeftPos;
    int frontRightPos;
    int backLeftPos;
    int backRightPos;

    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftPos = 0;
        frontRightPos = 0;
        backLeftPos = 0;
        backRightPos = 0;


        waitForStart();
        //850 Ticks for 90 degrees
        //9.44 Tick/Degree
        //14.38 Tick/cm

        drive(992.22, 992.22, 992.22, 992.22, 0.3);
        sleep(300);
        drive(-143.8, -143.9, -143.8, -143.8, 0.3);
        drive(850,-850, -850, 850, 0.3);
        drive(1610.56, 1610.56, 1610.56, 1610.56, 0.3);
        drive(850, -850, -850, 850, 0.3);
        drive(600, 600, 600, 600, 0.3);
        drive(-600, -600, -600, -600, 0.3);
        drive(850, -850, -850, 850, 0.3);
        drive(1610.56, 1610.56, 1610.56, 1610.56, 0.3);
        //turn right 90 degrees
        drive(850, -850, -850, 850, 0.3);
        drive(143.8, 143.8, 143.8, 143.8, 0.3);
        sleep(300);
        //back
        drive(-143.8, -143.8, -143.8, -143.8, 0.3);
        drive(850, -850, -850, 850, 0.3);
        drive(1000, 1000, 1000, 1000, 0.3);
        //strafe left
        drive(1006.6, -1006.6, 1006.6, -1006.6, 0.3);
        //forward
        drive(575.2, 575.2, 575.2, 575.2, 0.3);
        //strafe right
        drive(-1653.7, 1653.7, -1653.7, 1653.7, 0.3);


    }

    private void drive(double bLeftTarget, double bRightTarget, double fRightTarget, double fLeftTarget, double speed ) {
        frontLeftPos += fLeftTarget;
        frontRightPos += fRightTarget;
        backLeftPos += bLeftTarget;
        backRightPos += bRightTarget;

        frontLeft.setTargetPosition(frontLeftPos);
        frontRight.setTargetPosition(frontRightPos);
        backLeft.setTargetPosition(backLeftPos);
        backRight.setTargetPosition(backRightPos);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while(opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
            idle();
            // sleep(1000);

            //frontLeft.setPower(0);
            //  frontRight.setPower(0);
            //  backLeft.setPower(0);
            //  backRight.setPower(0);
        }
    }
}
