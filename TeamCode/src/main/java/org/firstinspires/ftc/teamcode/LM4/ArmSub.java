package org.firstinspires.ftc.teamcode.LM4;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class ArmSub {

    private DcMotorEx slidesJoint, slides;
    private Servo claw;


    public static double clawOpenPos = 0.5;
    public static double clawClosePos = 0.26;


    public static double speedDivider = 3;

    //slide hard stops
    public static int maxSlidePos = 4800;
    public static int minSlidePos = 0;

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
    public static int armInitPos = 0;
    public static int armIntakePos = 735;
    public static int armDeliverPos = 460;
    public static int armDriveAroundPos = 170;

    public ArmSub(HardwareMap hardwareMap, Telemetry telemetry) {

        claw = hardwareMap.get(Servo.class, "claw");
        slidesJoint = hardwareMap.get(DcMotorEx.class, "slidesJoint");
        slides = hardwareMap.get(DcMotorEx.class, "slides");

        slidesJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slidesJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesJoint.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int slidesJointTarget = 0;
        slidesJoint.setTargetPosition(slidesJointTarget);
        slidesJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        int slidesTargetPos = 0;
        slides.setTargetPosition(slidesTargetPos);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();


        lastTime = System.currentTimeMillis();

    }


    public void runArmToPos(int targetPos, double speed){
        slidesJoint.setTargetPosition(targetPos);
        slidesJoint.setPower(speed);

    }


    public void setSlidesJointPos(double targetPos, int speedDiv) {
        setpoint = targetPos;
        speedDivider = speedDiv;

    }

    public void setSlides(int targetPos) {

        if (targetPos > maxSlidePos) {
            targetPos = maxSlidePos;

        } else if (targetPos < minSlidePos) {
            targetPos = minSlidePos;
        }

        slides.setTargetPosition(targetPos);
        slides.setPower(1);
    }

    public void clawOpen(){
        claw.setPosition(clawOpenPos);

    }
    public void clawClose() {
        claw.setPosition(clawClosePos);

    }

    public void update() {
        double currentPosition = slidesJoint.getCurrentPosition();
        double error = setpoint - currentPosition;

        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        double pTerm = Kp * error;
        integralSum += error * deltaTime;
        double iTerm = Ki * integralSum;
        double derivative = (error - lastError) / deltaTime;
        double dTerm = Kd * derivative;

        double output = Math.max(-1, Math.min(pTerm + iTerm + dTerm, 1));
        slidesJoint.setPower(output / speedDivider);

        lastError = error;

    }

}
