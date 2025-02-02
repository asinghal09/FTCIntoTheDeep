package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name="Servo Button Control", group="TeleOp")
public class autodemo extends OpMode {

    // Declare servo
    Servo myServo;

    // Initialize the servo
    @Override
    public void init() {
        myServo = hardwareMap.get(Servo.class, "claw"); // Replace "servo" with your servo's name
    }

    // Loop to control the servo position when button A is pressed
    @Override
    public void loop() {
        // Check if button A is pressed
        if (gamepad1.a) {
            myServo.setPosition(1.0); // Move the servo to the maximum position (1.0 is the full position)
        } else {
            myServo.setPosition(0.0); // Move the servo to the minimum position (0.0 is the rest position)
        }

        // Telemetry to monitor the servo position
        telemetry.addData("Servo Position", myServo.getPosition());
        telemetry.update();
    }
}