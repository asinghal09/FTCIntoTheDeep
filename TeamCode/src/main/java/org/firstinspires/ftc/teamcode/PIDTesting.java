package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class PIDTesting extends LinearOpMode {

        // PID coefficients
        public static double Kp = 0.01;   // Proportional Gain
        public static double Ki = 0.0;    // Integral Gain
        public static double Kd = 0.0;    // Derivative Gain

        // Target position for the motor
        public static int targetPosition = 1000;

        // Integral and previous error for PID calculation
        private double integralSum = 0;
        private double lastError = 0;

        // Time tracking for PID calculation
        private long lastTime;

        @Override
        public void runOpMode() {
            // FTC Dashboard
            FtcDashboard dashboard = FtcDashboard.getInstance();

            // Motor initialization
            DcMotorEx armLeft = hardwareMap.get(DcMotorEx.class, "armLeft");
            DcMotorEx armRight = hardwareMap.get(DcMotorEx.class, "armRight");
            armRight.setDirection(DcMotorSimple.Direction.REVERSE);
            armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armLeft.setTargetPosition(targetPosition);
            armRight.setTargetPosition(targetPosition);
            armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);




            waitForStart();

            lastTime = System.currentTimeMillis();

            while (opModeIsActive()) {
                // Get current motor position
                int currentPosition = armLeft.getCurrentPosition();

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

                // Set motor power based on PID output
                armLeft.setPower(output);
                armRight.setPower(output);

                // Send telemetry to the dashboard
                telemetry.addData("Target", targetPosition);
                telemetry.addData("Current Position", currentPosition);
                telemetry.addData("Error", error);
                telemetry.addData("PID Output", output);
                telemetry.addData("Kp", Kp);
                telemetry.addData("Ki", Ki);
                telemetry.addData("Kd", Kd);
                telemetry.update();
            }
        }
    }
