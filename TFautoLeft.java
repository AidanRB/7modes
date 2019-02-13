package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "TF Left", group = "Concept")

public class TFautoLeft extends OpMode {
    private Qbert robot = new Qbert();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AaIK0f//////AAAAGTsgmHszM030skFBvcnAJlNSWaH7oKLZhxAZeCi7ToBGSKkO7T3EvzsRVYQdyDp2X+TFK6TQs+3WoCHkZXDYPQd87f77D6kvcBr8zbJ07Fb31UKiXdUBvX+ZQSV3kBhdAoxhfMa0WPgys7DYaeiOmM49CsNra7nVh05ls0th3h07wwHz3s/PBZnQwpbfr260CDgqBv4e9D79Wg5Ja5p+HAOJvyqg2r/Z5dOyRvVI3f/jPBRZHvDgDF9KTcuJAPoDHxfewmGFOFtiUamRLvcrkK9rw2Vygi7w23HYlzFO7yap+jUk1bv0uWNc0j5HPJDAjqa2ijBN9aVDrxzmFJml5WMA3GJJp8WOd9gkGhtI/BIo";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private String goldPosition = "left";

    public void init() {
        initVuforia();

        if(ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }
        else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        tfod.activate();

        robot.init(hardwareMap);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void init_loop() {
        TelemetryPacket packet = new TelemetryPacket();

        if(tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if(updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if(updatedRecognitions.size() > 0) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    int i = 0;
                    for(Recognition recognition : updatedRecognitions) {
                        if(recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        }
                        else if(silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        }
                        else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                        if(recognition.getTop() > 200 && recognition.getLabel() == LABEL_GOLD_MINERAL) {
                            telemetry.addData(recognition.getLabel(), "left, top, conf = %.0f, %.0f, %.2f", recognition.getLeft(), recognition.getTop(), recognition.getConfidence());
                            //packet.put(recognition.getLabel(), "left, top, conf = %.0f, %.0f, %.2f", recognition.getLeft(), recognition.getTop(), recognition.getConfidence());
                            packet.put(String.format("%s %d left", recognition.getLabel(), i), recognition.getLeft());
                            packet.put(String.format("%s %d top", recognition.getLabel(), i), recognition.getTop());
                            packet.put(String.format("%s %d confidence", recognition.getLabel(), i), recognition.getConfidence());
                            if(recognition.getConfidence() > 0.8) {
                                if(recognition.getLeft() > 600) {
                                    goldPosition = "right";
                                }
                                else {
                                    goldPosition = "middle";
                                }
                            }
                        }

                        //if(recognition.getLeft())

                        i++;
                    }
                }
                telemetry.update();
            }
        }
    }

    public void start() {
        turn(90);
        // Lower and let go
        robot.mark.setPosition(0.7);
        robot.lift.setTargetPosition(robot.lift.getCurrentPosition() - 22000);
        while(robot.lift.isBusy()) {
        }
        robot.grab.setPower(-1);
        pause(2000);
        robot.grab.setPower(0);
    }

    public void loop() {
    }

    public void stop() {
        tfod.shutdown();
    }

    //@Override

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void pause(long time) {
        try {
            Thread.sleep(time);
        } catch(Exception e) {
        }
    }

    private void driveForward(double speed, double distance) {
        robot.one.setPower(speed);
        robot.two.setPower(speed);
        robot.three.setPower(-speed);
        robot.four.setPower(-speed);

        int initial = robot.one.getCurrentPosition();
        while(robot.one.getCurrentPosition() < initial + distance * 1075.2 / 3.1415 / 5) {
            telemetry.addData("one", robot.one.getCurrentPosition());
            telemetry.update();
        }
        stopWheels();
    }

    private void driveBackward(double speed, double distance) {
        robot.one.setPower(-speed);
        robot.two.setPower(-speed);
        robot.three.setPower(speed);
        robot.four.setPower(speed);

        int initial = robot.one.getCurrentPosition();
        while(robot.one.getCurrentPosition() > initial - distance * 1075.2 / 3.1415 / 5) {
            telemetry.addData("one", robot.one.getCurrentPosition());
            telemetry.update();
        }
        stopWheels();
    }

    private void driveLeft(double speed, double distance) {
        robot.one.setPower(speed);
        robot.two.setPower(-speed);
        robot.three.setPower(-speed);
        robot.four.setPower(speed);

        int initial = robot.one.getCurrentPosition();
        while(robot.one.getCurrentPosition() < initial + distance * 1075.2 / 3.1415 / 5) {
        }
        stopWheels();
    }

    private void driveRight(double speed, double distance) {
        robot.one.setPower(-speed);
        robot.two.setPower(speed);
        robot.three.setPower(speed);
        robot.four.setPower(-speed);

        int initial = robot.one.getCurrentPosition();
        while(robot.one.getCurrentPosition() > initial - distance * 1075.2 / 3.1415 / 5) {
        }
        stopWheels();
    }

    /*private void turnEnc(double speed, double distance) {
        robot.one.setPower(speed);
        robot.two.setPower(speed);
        robot.three.setPower(speed);
        robot.four.setPower(speed);
        zero = robot.one.getCurrentPosition();
        if(distance < 0) {
            while(robot.one.getCurrentPosition)
        }
    }*/

    private void turn(double turnAmount) {
        robot.updateGyro(5);
        float currentAngle = robot.angles.thirdAngle;
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("angle", currentAngle);
        dashboard.sendTelemetryPacket(packet);
        double setangle = currentAngle - turnAmount;
        double turn = 0;
        while(true) {
            robot.updateGyro(5);
            currentAngle = robot.angles.thirdAngle;
            packet = new TelemetryPacket();
            packet.put("currentAngle", currentAngle);
            packet.put("setangle", setangle);
            packet.put("turn", turn);
            dashboard.sendTelemetryPacket(packet);
            turn = -wrap(currentAngle - setangle) / 180;     // Get needed correction
            /*else {                                          // If manually turning:
                setangle = currentAngle;             // Set return angle
            }*/
            if(Math.abs(turn) < 0.05)
                turn = 0;                 // don't bother if you're super close
            if(turn < 0.15 && turn >0.05) turn = 0.15;         // if .05-.15, use .15 power
            if(turn > -0.15 && turn < -0.05)
                turn = -0.15;      // ditto for negative; avoids squeeking

            robot.one.setPower(turn);
            robot.two.setPower(turn);
            robot.three.setPower(turn);
            robot.four.setPower(turn);

            if(turn == 0) {      // If it was turning and finished:
                break;
            }
        }
    }

    private void stopWheels() {
        robot.one.setPower(0);
        robot.two.setPower(0);
        robot.three.setPower(0);
        robot.four.setPower(0);
    }

    private double wrap(double input) {
        while(Math.abs(input) > 180) {
            if(input < -180) {
                input += 360;
            }
            else {
                input -= 360;
            }
        }
        return input;
    }
}
