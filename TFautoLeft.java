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

@Autonomous(name = "TF Crater", group = "Concept")

public class TFautoLeft extends TFauto {

    public void runOpMode() {
        super.runOpMode();

        // Lower and let go
        //robot.mark.setPosition(0.0);
        robot.lift.setTargetPosition(robot.lift.getCurrentPosition() - 18500);
        robot.lift.setPower(1);
        while(robot.lift.isBusy()) {
            opModeIsActive();
        }
        robot.latch.setPower(-1);
        pause(2500);
        robot.latch.setPower(0);

        double speed = 0.8;
        double middledist = 18;
        double sidedist = 23;
        double sideturn = 35;
        long pausetime = 300;
        driveBackward(speed, 1);
        turnAbs(0);
        driveBackward(speed, 4);
        //pause(1000);
        switch(goldPosition) {
            case "middle":
                driveBackward(speed, middledist);
                //pause(pausetime);
                //driveForward(speed, 10);
                //turnAbs(0);
                //driveRight(speed, 12);
                break;
            case "left":
                pause(pausetime);
                turnAbs(sideturn);
                driveBackward(speed, sidedist);
                //pause(pausetime);
                //driveForward(speed, 15);
                break;
            case "right":
                pause(pausetime);
                turnAbs(-sideturn);
                driveBackward(speed, sidedist);
                //pause(pausetime);
                //driveForward(speed, 15);
                //turnAbs(0);
                //driveRight(speed, 30);
                break;
        }
        turnAbs(90);
        //driveBackward();
        robot.intakearm.setTargetPosition(-800);
        robot.intakearm.setPower(0.4);
        while(robot.intakearm.isBusy() && opModeIsActive()) {}
    }
}
