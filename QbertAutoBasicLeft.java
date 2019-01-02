package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Basic Left", group="Basic")

public class QbertAutoBasicLeft extends LinearOpMode {
    private Qbert robot = new Qbert();
    
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();

        // run until the end of the match (driver presses STOP)
        telemetry.addData("current lift position", robot.lift.getCurrentPosition());
        telemetry.update();
        /*robot.lift.setTargetPosition(robot.lift.getCurrentPosition() - 10000);
        robot.lift.setPower(100);
        while(robot.lift.isBusy()){}
        robot.lift.setPower(0);
        pause(1000);
        driveForward(0.25, -5);
        pause(1000);*/
        turn(-50, 1);
    }
    
    private void pause(long time) {
        try {
            Thread.sleep(time);
        } catch(Exception e) {
        }
    }
    
    private void driveForward(double speed, double distance){
        robot.one.setPower(speed);
        robot.two.setPower(speed);
        robot.three.setPower(-speed);
        robot.four.setPower(-speed);
        
        int initial = robot.one.getCurrentPosition();
        while(robot.one.getCurrentPosition() < initial + distance*1075.2/3.1415/5) {}
        stopWheels();
    }
    
    private void turn(double speed, double angle) {
        robot.one.setPower(speed);
        robot.two.setPower(speed);
        robot.three.setPower(speed);
        robot.four.setPower(speed);
        while(robot.angles.thirdAngle < robot.angles.thirdAngle - angle) {}
        stopWheels();
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
