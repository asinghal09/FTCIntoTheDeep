package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;





public class AutoMethods{
    private DcMotor frontLeft, frontRight, backLeft, backRight;



    public AutoMethods(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight){
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;

        this.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void stopMotors(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    public void driveForward(double power, int time) throws InterruptedException {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        Thread.sleep(time);

        stopMotors();

    }
    public void driveBackward(double power, int time) throws InterruptedException {
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);

        Thread.sleep(time);

        stopMotors();

    }
    public void strafe(boolean right, double power, int time) throws InterruptedException{
        if(right){
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(-power);
            backRight.setPower(power);

            Thread.sleep(time);

            stopMotors();
            Thread.sleep(200);

        } else if(!right){
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(-power);

            Thread.sleep(time);

            stopMotors();
            Thread.sleep(200);
        }
    }
    public void turnInPlace(double angle) throws InterruptedException{

        if (angle <= 180) {
            int time = (int)(angle * 7);
            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(0.5);
            backRight.setPower(-0.5);

            Thread.sleep(time);

            stopMotors();
            Thread.sleep(200);
        } else if (angle > 181) {
            angle = 360-angle;
            int time = (int)(angle * 6.8);
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);

            Thread.sleep(time);

            stopMotors();
            Thread.sleep(200);
        }
    }



}

