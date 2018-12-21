package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="One", group="Mecanum")
public class OneRobotToFindThem extends OpMode{

    /* Declare OpMode members. */
    private OneRobotToRuleThemAll robot = new OneRobotToRuleThemAll();
    //float Ch1,Ch2,Ch3,Ch4,FrontLeft,BackLeft,FrontRight,BackRight;
    //float r,theta;
    float Ch1, Ch3, Ch4;
    float aragorn, legolas, gimli, boromir;
    double theta, r;
    boolean fieldcentric = true;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        telemetry.addData("Robot", "ready");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        robot.updateGyro(5);
        
        Ch1 = -gamepad1.right_stick_x;
        Ch3 = -gamepad1.left_stick_y;
        Ch4 = -gamepad1.left_stick_x;
        
        theta = Math.atan2(Ch3, Ch4);
        r = Math.sqrt((Ch3*Ch3) + (Ch4*Ch4));
        
        telemetry.addData("theta", theta);
        telemetry.addData("r", r);
        telemetry.addData("Ch3", Ch3);
        telemetry.addData("Ch4", Ch4);
        
        telemetry.addData("firstAngle", robot.angles.firstAngle);
        telemetry.addData("secondAngle", robot.angles.secondAngle);
        telemetry.addData("thirdAngle", robot.angles.thirdAngle);
        
        theta += Math.toRadians(robot.angles.thirdAngle + 180);
        
        if(fieldcentric){
            Ch3 = (float) Math.sin(theta);
            Ch4 = (float) Math.cos(theta);
            
            Ch3 *= r;
            Ch4 *= r;
        }
        
        telemetry.addData("Ch3", Ch3);
        telemetry.addData("Ch4", Ch4);
        
        aragorn = Ch3 + Ch1 + Ch4;
        legolas = Ch3 + Ch1 - Ch4;
        gimli = Ch3 - Ch1 - Ch4;
        boromir = Ch3 - Ch1 + Ch4;
        
        setAllS(aragorn, legolas, gimli, boromir);

        if(gamepad1.a)
        robot.aragorn.setPower(1);
        else if(gamepad1.b)
        robot.boromir.setPower(1);
        else if(gamepad1.y)
        robot.gimli.setPower(1);
        else if(gamepad1.x)
        robot.legolas.setPower(1);

        if(gamepad1.left_bumper) {
            fieldcentric = true;
        } else if(gamepad1.right_bumper) {
            fieldcentric = false;
        }

        telemetry.addData("fieldcentric", fieldcentric);
        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private double si(double joyVal, double divBy) {
        /**
         * This function scales the input.
         * It makes it so that when you push the joystick forward,
         * the robot doesn't respond in a linear way.
         * When you push it a little bit, you have more precision control over speed.
         * Pushing all the way overrides the math and jumps to 1.
         */
        if(joyVal>=1) return 1;
        else if(joyVal<=-1) return -1;
        else return -Math.log(-joyVal + 1) / divBy;
    }
    
    /*private void timeDrive(float x, float y,  int time) {
        float fl = x + y;
        float bl = x - y;
        float fr = x - y;
        float br = x + y;
        
        setAll(fl, bl, fr, br);
        wait(1000);
        setAll(0, 0, 0, 0);
    }*/
    
    private void setAll(float fl, float fr, float bl, float br) {
        robot.aragorn.setPower(fl);
        robot.legolas.setPower(fr);
        robot.gimli.setPower(bl);
        robot.boromir.setPower(br);
    }
    
    private void setAllS(float fl, float fr, float bl, float br) {
        robot.aragorn.setPower(Range.clip(si(fl, 1.5), -1, 1));
        robot.legolas.setPower(Range.clip(si(fr, 1.5), -1, 1));
        robot.boromir.setPower(Range.clip(si(bl, 1.5), -1, 1));
        robot.gimli.setPower(Range.clip(si(br, 1.5), -1, 1));
    }
    
    private void wait(int time) {
        try {
            Thread.sleep(time);
        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}





