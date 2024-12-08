package org.firstinspires.ftc.teamcode;
public class PIDController {
    private double kp; //proportional gain
    private double ki; //integral gain
    private double kd; //derivative gain

    private double setpoint; //desired target value
    private double integral; //accumulated error
    private double lastError; //last error value
    private double lastTime; //time of last update

    private double currentValue; //measurement of current state of robot

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.setpoint = 0;
        this.integral = 0;
        this.lastError = 0;
        this.lastTime = System.currentTimeMillis();
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint; // Only set the target
    }
    public double calculate(double currentValue) {
        double currentTime =System.currentTimeMillis();
        double deltaTime = (currentTime - lastTime) / 1000.0; //seconds

        double error = setpoint - currentValue;
        integral += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;

        double output = kp * error + ki * integral + kd * derivative;

        lastError = error;
        lastTime = currentTime;

        return output;
    }
}