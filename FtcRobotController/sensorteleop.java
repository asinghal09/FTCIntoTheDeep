package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp


public class sensorteleop extends LinearOpMode {
    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;

    private DcMotor clawMotor;  // Motor controlling the claw (adjust based on your setup)

    @Override
    public void runOpMode() {
        // Initialize the hardware
        clawMotor = hardwareMap.dcMotor.get("clawMotor");

        // Initialize Vuforia and TFOD
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();  // Start the TFOD system
        }

        waitForStart();

        while (opModeIsActive()) {
            // Detect objects
            if (tfod != null) {
                List<Recognition> recognitions = tfod.getUpdatedRecognitions();

                if (recognitions != null) {
                    for (Recognition recognition : recognitions) {
                        telemetry.addData("Label", recognition.getLabel());
                        telemetry.addData("Confidence", recognition.getConfidence());
                        telemetry.addData("Position", "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                    }
                    telemetry.update();
                }
            }

            // Check if the "X" button is pressed to trigger the alignment
            if (gamepad1.x) {
                alignClawToSample();
            }

            // Add other TeleOp controls here for driving, etc.
            // e.g., drive with gamepad1, lift arms, etc.
        }
    }

    // Initialize Vuforia
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "YOUR_VUFORIA_LICENSE_KEY";  // Replace with your Vuforia license key
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");  // Set the webcam
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
    }

    // Initialize TensorFlow Object Detection (TFOD)
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters parameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(parameters, vuforia);
        tfod.loadModelFromAsset("sample_model.tflite", "Sample1", "Sample2");  // Replace with your model
    }

    // Align the claw based on the detected sample's position
    private void alignClawToSample() {
        if (tfod != null) {
            List<Recognition> recognitions = tfod.getUpdatedRecognitions();
            if (recognitions != null && !recognitions.isEmpty()) {
                Recognition recognition = recognitions.get(0);  // Get the first detected sample

                // Calculate the center of the detected object
                double centerX = (recognition.getLeft() + recognition.getRight()) / 2;
                double centerY = (recognition.getTop() + recognition.getBottom()) / 2;

                // Print the position data (for debugging purposes)
                telemetry.addData("Center X", centerX);
                telemetry.addData("Center Y", centerY);
                telemetry.update();

                // Adjust the claw motor to align with the sample based on its position
                if (centerX < 320) {
                    // If the sample is to the left, move the claw to the left
                    clawMotor.setPower(0.5);  // Adjust power and duration based on your setup
                } else if (centerX > 320) {
                    // If the sample is to the right, move the claw to the right
                    clawMotor.setPower(-0.5);  // Negative power to move in the opposite direction
                } else {
                    // Stop the motor when the sample is centered
                    clawMotor.setPower(0);
                }
            }
        }
    }
