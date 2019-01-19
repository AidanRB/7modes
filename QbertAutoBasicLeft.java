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
        robot.lift.setTargetPosition(robot.lift.getCurrentPosition() - 32000);
        robot.lift.setPower(100);
        while(robot.lift.isBusy()){}
        //robot.lift.setPower(0);
        
        robot.grab.setPower(-1);
        pause(1500);
        robot.grab.setPower(0);
        
        //robot.lift.setTargetPosition(robot.lift.getCurrentPosition() + 32000);
        
        driveBackward(0.7, 17); // away from lander
        pause(200);
        driveRight(0.7, 18); // to side of field
        pause(200);
        driveBackward(0.7, 5); // knock off cube/whatever
        pause(200);
        driveRight(0.7, 18); // to side of field
        pause(200);
        turnCounter(0.5, 48); // to facing floor goal
        pause(200);
        driveRight(0.7, 55); // to floor goal
        pause(200);
        robot.hand.setPower(-1); // drop marker
        pause(2000);
        robot.hand.setPower(0);
        driveLeft(0.7, 70); // 
        robot.arm.setPower(-0.2);
        pause(3000);
        robot.arm.setPower(0);
        
        while(robot.lift.isBusy()) {}
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
        while(robot.one.getCurrentPosition() < initial + distance*1075.2/3.1415/5) {
            telemetry.addData("one", robot.one.getCurrentPosition());
            telemetry.update();
        }
        stopWheels();
    }
    
    private void driveBackward(double speed, double distance){
        robot.one.setPower(-speed);
        robot.two.setPower(-speed);
        robot.three.setPower(speed);
        robot.four.setPower(speed);
        
        int initial = robot.one.getCurrentPosition();
        while(robot.one.getCurrentPosition() > initial - distance*1075.2/3.1415/5) {     
            telemetry.addData("one", robot.one.getCurrentPosition());
            telemetry.update();
        }
        stopWheels();
    }

    private void driveLeft(double speed, double distance){
        robot.one.setPower(speed);
        robot.two.setPower(-speed);
        robot.three.setPower(-speed);
        robot.four.setPower(speed);
        
        int initial = robot.one.getCurrentPosition();
        while(robot.one.getCurrentPosition() < initial + distance*1075.2/3.1415/5) {}
        stopWheels();
    }
    
    private void driveRight(double speed, double distance){
        robot.one.setPower(-speed);
        robot.two.setPower(speed);
        robot.three.setPower(speed);
        robot.four.setPower(-speed);
        
        int initial = robot.one.getCurrentPosition();
        while(robot.one.getCurrentPosition() > initial - distance*1075.2/3.1415/5) {}
        stopWheels();
    }
    
    private void turnCounter(double speed, double angle) {
        robot.one.setPower(speed);
        robot.two.setPower(speed);
        robot.three.setPower(speed);
        robot.four.setPower(speed);
        
        robot.updateGyro(5);
        //angle -= 5;
        double leeway = 40;
        double current = robot.angles.thirdAngle;
        
        double lesser = current + angle;
        double greater = current + angle + leeway;
        if(greater < 180) {
            while(current < lesser) {
                robot.updateGyro(5);
                current = robot.angles.thirdAngle;
            }
        }
        else if(greater > 180 && lesser < 180)  {
            greater = wrap(greater);
            while(!(current > lesser || current < greater)) {
                robot.updateGyro(5);
                current = robot.angles.thirdAngle;
            }
        }
        else if(lesser > 180) {
            greater = wrap(greater);
            lesser = wrap(lesser);
            while(!(current > lesser && current < greater)) {
                robot.updateGyro(5);
                current = robot.angles.thirdAngle;
            }
        }
        else {
            telemetry.addData("","wat");
            telemetry.update();
            pause(30000);
        }
        
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
