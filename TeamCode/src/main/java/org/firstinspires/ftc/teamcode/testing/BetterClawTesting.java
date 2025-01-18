package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@Config
@TeleOp
public class BetterClawTesting extends LinearOpMode{
    Servo claw, joint, spinny;
    public static double clawPos = 0.5;
    public static double jointPos = 0.5;
    public static double spinnyPos = 0.5;

    public void runOpMode(){
        claw = hardwareMap.get(Servo.class, "claw");
        joint = hardwareMap.get(Servo.class, "joint");
        spinny = hardwareMap.get(Servo.class, "spinny");

        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.a)
                claw.setPosition(clawPos);
            if (gamepad1.b)
                joint.setPosition(jointPos);
            if (gamepad1.x)
                spinny.setPosition(spinnyPos);
        }

    }


}
