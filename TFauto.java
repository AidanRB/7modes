package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class TFauto extends LinearOpMode {
    Qbert robot = new Qbert();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AaIK0f//////AAAAGTsgmHszM030skFBvcnAJlNSWaH7oKLZhxAZeCi7ToBGSKkO7T3EvzsRVYQdyDp2X+TFK6TQs+3WoCHkZXDYPQd87f77D6kvcBr8zbJ07Fb31UKiXdUBvX+ZQSV3kBhdAoxhfMa0WPgys7DYaeiOmM49CsNra7nVh05ls0th3h07wwHz3s/PBZnQwpbfr260CDgqBv4e9D79Wg5Ja5p+HAOJvyqg2r/Z5dOyRvVI3f/jPBRZHvDgDF9KTcuJAPoDHxfewmGFOFtiUamRLvcrkK9rw2Vygi7w23HYlzFO7yap+jUk1bv0uWNc0j5HPJDAjqa2ijBN9aVDrxzmFJml5WMA3GJJp8WOd9gkGhtI/BIo";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public String goldPosition = "left";

    public void runOpMode() {
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
        robot.intakearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(!opModeIsActive()) {  // pre-opmode loop
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

                            if(recognition.getTop() > 300 && recognition.getLabel() == LABEL_GOLD_MINERAL) {
                                telemetry.addData(recognition.getLabel(), "left, top, conf = %.0f, %.0f, %.2f", recognition.getLeft(), recognition.getTop(), recognition.getConfidence());
                                //packet.put(recognition.getLabel(), "left, top, conf = %.0f, %.0f, %.2f", recognition.getLeft(), recognition.getTop(), recognition.getConfidence());
                                packet.put(String.format("%s %d left", recognition.getLabel(), i), recognition.getLeft());
                                packet.put(String.format("%s %d top", recognition.getLabel(), i), recognition.getTop());
                                packet.put(String.format("%s %d confidence", recognition.getLabel(), i), recognition.getConfidence());
                                goldPosition = "left";
                                if(recognition.getConfidence() > 0.7) {
                                    if(recognition.getLeft() > 600) {
                                        goldPosition = "right";
                                    }
                                    else {
                                        goldPosition = "middle";
                                    }

                                    i++;
                                }
                            }
                        }
                        packet.put("mineral position", goldPosition);
                        telemetry.addData("mineral position", goldPosition);
                    }
                    telemetry.update();
                    dashboard.sendTelemetryPacket(packet);
                }
            }
        }   // end of pre-opmode loop
    }

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

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    void pause(long time) {
        try {
            Thread.sleep(time);
        } catch(Exception e) {
        }
    }

    void driveForward(double speed, double distance) {
        robot.one.setPower(speed);
        robot.two.setPower(speed);
        robot.three.setPower(-speed);
        robot.four.setPower(-speed);

        int initial = robot.one.getCurrentPosition();
        float currentAngle;
        currentAngle = -robot.angles.secondAngle;
        float setangle = currentAngle;
        TelemetryPacket packet;
        double turn = 0;
        while(robot.one.getCurrentPosition() < initial + distance * 1075.2 / 3.1415 / 5) {
            robot.updateGyro(5);
            currentAngle = -robot.angles.secondAngle;
            packet = new TelemetryPacket();
            packet.put("currentAngle", currentAngle);
            packet.put("setangle", setangle);
            packet.put("turn", turn);
            dashboard.sendTelemetryPacket(packet);
            turn = -wrap(currentAngle - setangle) / 180 * 1.6;     // Get needed correction
            turn *= 1.5;

            robot.one.setPower(speed + turn);
            robot.two.setPower(speed + turn);
            robot.three.setPower(-speed + turn);
            robot.four.setPower(-speed + turn);

            telemetry.addData("one", robot.one.getCurrentPosition());
            telemetry.update();
        }
        stopWheels();
    }

    void driveBackward(double speed, double distance) {
        robot.one.setPower(-speed);
        robot.two.setPower(-speed);
        robot.three.setPower(speed);
        robot.four.setPower(speed);

        int initial = robot.one.getCurrentPosition();
        float currentAngle;
        currentAngle = -robot.angles.secondAngle;
        float setangle = currentAngle;
        TelemetryPacket packet;
        double turn = 0;
        while(robot.one.getCurrentPosition() > initial - distance * 1075.2 / 3.1415 / 5) {
            robot.updateGyro(5);
            currentAngle = -robot.angles.secondAngle;
            packet = new TelemetryPacket();
            packet.put("currentAngle", currentAngle);
            packet.put("setangle", setangle);
            packet.put("turn", turn);
            dashboard.sendTelemetryPacket(packet);
            turn = -wrap(currentAngle - setangle) / 180 * 1.6;     // Get needed correction
            turn *= 1.5;

            robot.one.setPower(-speed + turn);
            robot.two.setPower(-speed + turn);
            robot.three.setPower(speed + turn);
            robot.four.setPower(speed + turn);

            telemetry.addData("one", robot.one.getCurrentPosition());
            telemetry.update();
            /*
             *
             */
        }
        stopWheels();
    }

    void driveLeft(double speed, double distance) {
        robot.one.setPower(speed);
        robot.two.setPower(-speed);
        robot.three.setPower(-speed);
        robot.four.setPower(speed);

        int initial = robot.one.getCurrentPosition();
        float currentAngle;
        currentAngle = -robot.angles.secondAngle;
        float setangle = currentAngle;
        TelemetryPacket packet;
        double turn = 0;
        while(robot.one.getCurrentPosition() < initial + distance * 1075.2 / 3.1415 / 5) {
            robot.updateGyro(5);
            currentAngle = -robot.angles.secondAngle;
            packet = new TelemetryPacket();
            packet.put("currentAngle", currentAngle);
            packet.put("setangle", setangle);
            packet.put("turn", turn);
            dashboard.sendTelemetryPacket(packet);
            turn = -wrap(currentAngle - setangle) / 180 * 1.6;     // Get needed correction
            turn *= 1.5;

            robot.one.setPower(speed + turn);
            robot.two.setPower(-speed + turn);
            robot.three.setPower(-speed + turn);
            robot.four.setPower(speed + turn);

            telemetry.addData("one", robot.one.getCurrentPosition());
            telemetry.update();
            /*
             *
             */
        }
        stopWheels();
    }

    void driveRight(double speed, double distance) {
        robot.one.setPower(-speed);
        robot.two.setPower(speed);
        robot.three.setPower(speed);
        robot.four.setPower(-speed);

        int initial = robot.one.getCurrentPosition();
        float currentAngle;
        currentAngle = -robot.angles.secondAngle;
        float setangle = currentAngle;
        TelemetryPacket packet;
        double turn = 0;
        while(robot.one.getCurrentPosition() > initial - distance * 1075.2 / 3.1415 / 5) {
            robot.updateGyro(5);
            currentAngle = -robot.angles.secondAngle;
            packet = new TelemetryPacket();
            packet.put("currentAngle", currentAngle);
            packet.put("setangle", setangle);
            packet.put("turn", turn);
            dashboard.sendTelemetryPacket(packet);
            turn = -wrap(currentAngle - setangle) / 180 * 1.6;     // Get needed correction
            turn *= 1.5;

            robot.one.setPower(-speed + turn);
            robot.two.setPower(speed + turn);
            robot.three.setPower(speed + turn);
            robot.four.setPower(-speed + turn);

            telemetry.addData("one", robot.one.getCurrentPosition());
            telemetry.update();
            /*
             *
             */
        }
        stopWheels();
    }

    /*void turnRel(double turnAmount) {
        robot.updateGyro(5);
        float currentAngle = -robot.angles.secondAngle;
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("angle", currentAngle);
        dashboard.sendTelemetryPacket(packet);
        double setangle = currentAngle - turnAmount;
        double turn = 0;
        turnAbs(setangle);
    }*/

    void turnAbs(double setangle) {
        float currentAngle;
        TelemetryPacket packet;
        double turn = 0;
        while(opModeIsActive()) {
            robot.updateGyro(5);
            currentAngle = -robot.angles.secondAngle;
            packet = new TelemetryPacket();
            packet.put("currentAngle", currentAngle);
            packet.put("setangle", setangle);
            packet.put("turn", turn);
            dashboard.sendTelemetryPacket(packet);
            turn = -wrap(currentAngle - setangle) / 180 * 1.6;     // Get needed correction
            turn *= 1.5;
            if(Math.abs(turn) < 0.04)
                turn = 0;                 // don't bother if you're super close
            if(turn < 0.15 && turn > 0.04) turn = 0.15;         // if .05-.15, use .15 power
            if(turn > -0.15 && turn < -0.04)
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

    void stopWheels() {
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