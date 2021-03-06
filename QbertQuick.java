package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
### OUTLINE OF CONCEPT ###

#Originals
x = joyx
y = joyy
wheels = [angle1, angle2, angle3, ...]

#To polar coordinates
r = min(sqrt(x*x + y*y), 1)
theta = atan2(x, y)

#Counter for gyroscope
theta += gyro

#Wheel power
for angle in wheels:
    #closeness to front: sin()
    power = cos(theta + angle)
*/

@TeleOp(name = "QbertQuick", group = "teleop")
public class QbertQuick extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Lift lift;
    private ScoringArm scoringarm;
    private double intakespeed = 0;
    private double armx = 1;
    private float recall = 0;
    /* Declare OpMode members. */
    private Qbert robot = new Qbert();
    // Defines wheel positions
    double[] wheels = new double[]{0, ((double) 1 / 2) * 3.1415, ((double) 1) * 3.1415, ((double) 3 / 2) * 3.1415};
    // Array for motor powers
    double[] powers = new double[]{0, 0, 0, 0};
    //Array for motor positions
    double[] encoders = new double[]{0, 0, 0, 0};
    // Current angle of gyroscope
    float currentAngle = 0;
    // Field centric drive; toggled with X
    boolean fieldcentric = true;
    // Relative zero for field-centric drive
    float zero = 0;
    // Self-righting; disables manual turning; toggles with Y
    boolean righteous = false;
    // Currently turning; used for exact turns
    boolean turning = false;
    // Speed multiplier from 1-10(11); adjusted up/down with A/B
    int speed = 10;
    // For detecting presses of X
    boolean fcswitch = false;
    // For detecting presses of Y
    boolean rgswitch = false;
    // For detecting presses of B
    boolean slswitch = false;
    // For detecting presses of A
    boolean fsswitch = false;
    // For detecting presses of back
    boolean ltswitch = false;
    // For detecting presses of start
    boolean rtswitch = false;

    // Initialize variables
    double x, y, turn, r, theta, setangle, delta;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        lift = new Lift(robot.lift);
        scoringarm = new ScoringArm(robot.scoringarm);
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
        TelemetryPacket packet = new TelemetryPacket();

        robot.updateGyro(5);    // Obtain new gyro information
        currentAngle = -robot.angles.secondAngle;


        // --- GAMEPAD INPUT ---
        turn = -gamepad1.right_stick_x;  // Use the right joy x for turning
        y = gamepad1.left_stick_y;      // Get left joy y
        x = -gamepad1.left_stick_x;     // Get left joy x
        if(gamepad1.left_bumper) {
            recall = currentAngle;
        }
        if(gamepad1.right_bumper) {
            turning = true;
            setangle = recall;
        }

        // --- TO POLAR COORDINATES ---
        r = Math.min(Math.sqrt(x * x + y * y), 1);      // Convert x,y to speed
        theta = Math.atan2(x, y);                   // and direction


        // --- DRIVE MODES ---
        if(speed == 11 && r != 0.0) {
            r = 1.0;
        }
        if(fieldcentric) {                                              // If field-centric:
            theta += Math.toRadians(currentAngle - zero);    // Compensate direction for gyro
        }
        if(righteous || turning) {                      // If self-righting or turning:
            if(turn == 0.0) {                               // If not manually turning:
                delta = currentAngle - setangle;     // Get needed correction
                turn = -wrap(delta) / 180 * 1.6;                       // Set turn to correction
            }
            else {                                          // If manually turning:
                setangle = currentAngle;             // Set return angle
            }
            if(Math.abs(turn) < 0.05)
                turn = 0;                 // don't bother if you're super close
            if(turn < 0.15 && turn > 0.05) turn = 0.15;         // if .05-.15, use .15 power
            if(turn > -0.15 && turn < -0.05)
                turn = -0.15;      // ditto for negative; avoids squeeking
        }
        if(turning && turn == 0) {      // If it was turning and finished:
            turning = false;                // Stop turning
        }


        // --- WHEEL POWERS ---
        r = r * r * r;                                  // Curve the speed exponentially
        r = r * speed / 10.0;                         // Adjust the translation speed
        turn = turn * speed / 10.0;
        for(int i = 0; i != wheels.length; i++) {                   // For all the wheels:
            powers[i] = (Math.sin(wheels[i] - theta) * r) + turn;  // calculate the desired speed
            // sin(wheel direction - desired direction finds speed), *r accounts
            // for joystick speed, +turn adds in turning with joystick/self
        }
        encoders[0] = robot.one.getCurrentPosition();
        encoders[1] = robot.two.getCurrentPosition();
        encoders[2] = robot.three.getCurrentPosition();
        encoders[3] = robot.four.getCurrentPosition();
        robot.one.setPower(powers[0]);     // Set motor powers
        robot.two.setPower(powers[1]);     // si(x, 1.5) applies curve, *(speed/10) applies speed
        robot.three.setPower(powers[2]);   // Range.clip() applies limit
        robot.four.setPower(powers[3]);


        // --- DRIVE MODE CONTROLS ---
        if(gamepad1.start) { // If start is pushed:
            zero = currentAngle;             // Reset forwards for field-centric
        }
        else if(!fcswitch && gamepad1.x) {              // Otherwise, if X is pushed:
            fieldcentric = !fieldcentric;               // Toggle field-centric
        }
        if(!rgswitch && gamepad1.y) {       // If Y is pushed:
            righteous = !righteous;                 // Toggle self-righting
            setangle = currentAngle;     // Set return angle
        }
        if(!slswitch && gamepad1.b && speed > 1) {      // If B is pushed and speed over 1:
            speed -= 2;                                 // decrease speed
        }
        if(!fsswitch && gamepad1.a && speed < 10) {     // If A is pushed and speed under 10:
            speed += 2;                                 // increase speed
        }
        if(!fsswitch && gamepad1.b && gamepad1.a && speed == 10) {
            speed = 11;
        }

        if(!ltswitch && gamepad1.right_stick_button) {                        // If back is pressed:
            setangle = wrap(currentAngle + 90);      // Set desired angle to current +90
            turning = true;                                     // Set to turning
        }
        if(!rtswitch && gamepad1.left_stick_button) {                       // If start is pressed:
            setangle = wrap(currentAngle - 90);      // Set desired angle to current -90
            turning = true;                                     // Set to turning
        }

        fcswitch = gamepad1.x;  // Store button states for next round
        rgswitch = gamepad1.y;
        slswitch = gamepad1.b;
        fsswitch = gamepad1.a;
        ltswitch = gamepad1.left_stick_button;
        rtswitch = gamepad1.right_stick_button;


        // --- SPECIAL MOTORS ---

        if(gamepad2.dpad_down) lift.down(1);
        if(gamepad2.dpad_up) lift.up(18500, 1);
        lift.check(!robot.liftbutton.getState());
        if(gamepad2.dpad_left) robot.latch.setPower(-1);
        else if(gamepad2.dpad_right) robot.latch.setPower(1);
        else robot.latch.setPower(0);

        if(gamepad2.right_bumper) robot.intake.setPower(1);
        else if(gamepad2.left_bumper) robot.intake.setPower(-1);
        else robot.intake.setPower(gamepad2.right_trigger * 0.4 - gamepad2.left_trigger * 0.2);



        // if the arm is all the way in and trying to go in, kill and reset it
        if(gamepad2.left_stick_y > 0 && !robot.intakebutton.getState()) {
            robot.intakearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else {
            robot.intakearm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // if the arm is all the way out and trying to go out, just kill it
        if(gamepad2.left_stick_y < 0 && robot.intakearm.getCurrentPosition() < -2500) {
            armx = 0;
        }
        else if(robot.intakearm.getCurrentPosition() > -800) {
            armx = 0.4;
        }
        else if(robot.intakearm.getCurrentPosition() < -2600 && gamepad2.left_stick_y < 0) {
            armx = 0.6;
        }
        else {
            armx = 1;
        }

        robot.intakearm.setPower(gamepad2.left_stick_y * armx);

        if(gamepad2.right_stick_button) scoringarm.stop();
        if(gamepad2.right_stick_y > 0.3) scoringarm.down(0.6);
        if(gamepad2.right_stick_y < -0.3) scoringarm.up(6200, 1.0);
        scoringarm.check(!robot.scoringbutton.getState());

        // --- DASHBOARD ---
        packet.put("fieldcentric (X to toggle)", fieldcentric);
        packet.put("righteous (Y to toggle)", righteous);
        packet.put("speed (A/B +/-)", speed / 10.0);

        packet.put("", null);

        packet.put("arm", gamepad2.left_stick_y);
        packet.put("slide", gamepad2.right_stick_y);

        packet.put("r", r);
        packet.put("theta", theta);
        for(int i = 0; i != wheels.length; i++) {
            packet.put("wheel " + Integer.toString(i + 1) + " power", powers[i]);
            packet.put("wheel " + Integer.toString(i + 1) + " position", encoders[i]);
        }
        if(fieldcentric || righteous || turning) {
            packet.put("", null);
            packet.put("angle", currentAngle);
        }
        if(fieldcentric) {
            packet.put("zero", zero);
        }
        if(righteous || turning) {
            packet.put("delta", delta);
            packet.put("delta%180", delta % 180);
            packet.put("turn", turn * 180);
        }

        packet.put("lift position", robot.lift.getCurrentPosition());
        packet.put("scoringarm position", robot.scoringarm.getCurrentPosition());
        packet.put("intakearm position", robot.intakearm.getCurrentPosition());

        dashboard.sendTelemetryPacket(packet);


        // --- TELEMETRY ---
        telemetry.addData("fieldcentric (X to toggle)", fieldcentric);
        telemetry.addData("righteous (Y to toggle)", righteous);
        telemetry.addData("speed (A/B +/-)", speed);

        telemetry.addData("", null);

        telemetry.addData("r", r);
        telemetry.addData("theta", theta);
        for(int i = 0; i != wheels.length; i++) {
            telemetry.addData("wheel " + Integer.toString(i + 1) + " power", powers[i]);
            telemetry.addData("wheel " + Integer.toString(i + 1) + " position", encoders[i]);
        }
        if(fieldcentric || righteous || turning) {
            telemetry.addData("", null);
            telemetry.addData("angle", currentAngle);
        }
        if(fieldcentric) {
            telemetry.addData("zero", zero);
        }
        if(righteous || turning) {
            telemetry.addData("delta", delta);
            telemetry.addData("delta%180", delta % 180);
            telemetry.addData("turn", turn * 180);
        }

        telemetry.addData("lift position", robot.lift.getCurrentPosition());

        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
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

class Lift {
    DcMotor lift;
    // Automatic lift variables
    boolean liftinit = true;        // if lift is running to the bottom at start
    boolean liftdown = true;        // if lift is running to the bottom otherwise
    int liftzero = 0;               // the bottom, in encoder clicks
    double speed = 1;

    Lift(DcMotor liftmotor) {
        lift = liftmotor;
    }

    public void down(double liftspeed) {
        speed = liftspeed;
        if(!liftinit) {
            liftdown = true;
        }
    }

    public void up(int top, double liftspeed) {
        if(!liftinit) {
            speed = liftspeed;
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(speed);
            lift.setTargetPosition(liftzero - top);
            liftdown = false;
        }
    }

    public void check(boolean pressed) {
        if(liftinit || liftdown) runDown(pressed);
        else if(!lift.isBusy()) {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(0);
        }
    }

    private void runDown(boolean pressed) {
        if(!pressed) {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(speed);
        }
        else {
            liftzero = lift.getCurrentPosition();
            liftinit = false;
            liftdown = false;
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setTargetPosition(liftzero);
        }
    }
}


class ScoringArm {
    DcMotor lift;

    //FtcDashboard dashboard = FtcDashboard.getInstance();

    // Automatic lift variables
    boolean liftinit = true;        // if lift is running to the bottom at start
    boolean liftdown = true;        // if lift is running to the bottom otherwise
    int liftzero = 0;               // the bottom, in encoder clicks
    double speed = 0.4;
    int target = 0;

    ScoringArm(DcMotor liftmotor) {
        lift = liftmotor;
    }

    public void down(double liftspeed) {
        speed = liftspeed;
        if(!liftinit) {
            liftdown = true;
        }
    }

    public void up(int top, double liftspeed) {
        if(!liftinit) {
            speed = liftspeed;
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(speed);
            target = liftzero - top;
            lift.setTargetPosition(target);
            liftdown = false;
        }
    }

    public void stop() {
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setPower(0);
    }

    public void check(boolean pressed) {
        //TelemetryPacket packet = new TelemetryPacket();
        if(liftinit || liftdown) runDown(pressed);
        else if(!lift.isBusy()) {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(0);
        }
        else if(lift.getCurrentPosition() < (target + 1000)) {
            lift.setPower(0.3);
        }

        /*packet.put("RunMode", lift.getMode());
        packet.put("Speed", lift.getPower());
        packet.put("Current", lift.getCurrentPosition());
        packet.put("Target", lift.getTargetPosition());
        packet.put("Pressed", pressed);

        dashboard.sendTelemetryPacket(packet);*/
    }

    private void runDown(boolean pressed) {
        if(!pressed) {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(speed);
        }
        else {
            liftzero = lift.getCurrentPosition();
            liftinit = false;
            liftdown = false;
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setTargetPosition(liftzero);
        }
    }
}
