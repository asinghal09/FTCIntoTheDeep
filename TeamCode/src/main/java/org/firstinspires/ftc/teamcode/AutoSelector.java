package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous
public class AutoSelector extends LinearOpMode {
    boolean blue, netZone;



    public void selection (){


        telemetry.addLine("Select your location:");
        telemetry.addLine("x/b for color");
        telemetry.addLine("y for net, a for observation");
        telemetry.update();



        if (gamepad1.x){
            blue = true; //blue
            netZone = true; //net
            telemetry.addData("Blue", blue );
            telemetry.addData("Net ", netZone);

        } else if (gamepad1.a) {
            blue = true;   //blue
            netZone = false; //observation
            telemetry.addData("Blue", blue );
            telemetry.addData("Net ", netZone);

        } else if (gamepad1.b) {
            blue = false;   //red
            netZone = true; //net
            telemetry.addData("Blue", blue );
            telemetry.addData("Net ", netZone);
        } else if (gamepad1.y) {
            blue = false;   //red
            netZone = false; //observation
            telemetry.addData("Blue", blue );
            telemetry.addData("Net ", netZone);

        }



    }


    public void runOpMode(){
        while (opModeInInit()){
            selection();
            sleep(100);

        }
        waitForStart();

    }
}


