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

@Autonomous(name = "TF Depot", group = "Concept")

public class TFautoRight extends TFauto {

    public void runOpMode() {
        super.runOpMode();

        // Lower and let go
        robot.mark.setPosition(0.7);
        robot.lift.setTargetPosition(robot.lift.getCurrentPosition() - 18500);
        robot.lift.setPower(1);
        while(robot.lift.isBusy()) {
            opModeIsActive();
        }
        robot.latch.setPower(-1);
        pause(2500);
        robot.latch.setPower(0);

        double speed = 0.8;
        double sidedist = 25;
        long pausetime = 300;
        driveBackward(speed, 1);
        turnAbs(0);
        driveBackward(speed, 4);

        //sample
        switch(goldPosition) {
            case "middle":
                driveBackward(speed, 40);
                pause(pausetime);
                break;
            case "left":
                pause(pausetime);
                turnAbs(45);
                driveBackward(speed, sidedist);
                turnAbs(-35);
                driveBackward(speed, sidedist);
                break;
            case "right":
                pause(pausetime);
                turnAbs(-45);
                driveBackward(speed, sidedist);
                turnAbs(35);
                driveBackward(speed, sidedist);
                break;
        }

        //claim
        turnAbs(-135);
        robot.mark.setPosition(0.0);
        pause(1000);

        //drive after
        switch(goldPosition) {
            case "middle":
                driveForward(speed, 10);
                break;
            case "left":
                break;
            case "right":
                driveForward(speed, 10);
                break;
        }
        turnAbs(-135);
        driveLeft(speed, 65);
        turnAbs(-135);
        robot.intakearm.setTargetPosition(-800);
        robot.intakearm.setPower(0.4);
        while(robot.intakearm.isBusy() && opModeIsActive()) {}
    }
}
