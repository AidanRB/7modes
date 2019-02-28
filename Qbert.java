package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This class is the hardware map of the robot.
 */
public class Qbert
{
    /* Public OpMode members. */
    public DcMotor one = null;
    public DcMotor two = null;
    public DcMotor three = null;
    public DcMotor four = null;
    public DcMotor lift = null;
    public CRServo latch = null;
    public DigitalChannel liftbutton = null;
    public DigitalChannel scoringbutton = null;
    public DigitalChannel intakebutton = null;
    public Servo mark = null;
    public DcMotor intakearm = null;
    public DcMotor intake = null;
    public DcMotor scoringarm = null;
    public Servo scoring = null;
    public BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    public float dangles[] = new float[3];

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public Qbert(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        one = ahwMap.dcMotor.get("one");
        two = ahwMap.dcMotor.get("two");
        three = ahwMap.dcMotor.get("three");
        four = ahwMap.dcMotor.get("four");
        lift = ahwMap.dcMotor.get("lift");
        liftbutton = ahwMap.digitalChannel.get("liftbutton");
        scoringbutton = ahwMap.digitalChannel.get("scoringbutton");
        intakebutton = ahwMap.digitalChannel.get("intakebutton");
        latch = ahwMap.crservo.get("latch");
        mark = ahwMap.servo.get("mark");
        intakearm = ahwMap.dcMotor.get("intakearm");
        intake = ahwMap.dcMotor.get("intake");
        scoringarm = ahwMap.dcMotor.get("scoringarm");
        scoring = ahwMap.servo.get("scoring");

        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        three.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        four.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakearm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scoringarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        three.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        four.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakearm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scoringarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //intakearm.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        
        imu = ahwMap.get(BNO055IMU.class, "imu 1");
        imu.initialize(parameters);
        
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        
        dangles[0] = angles.firstAngle;
        dangles[1] = angles.secondAngle;
        dangles[2] = angles.thirdAngle;
    }

    public void updateGyro(int damping) {
        float origX = angles.firstAngle;
        float origY = angles.secondAngle;
        float origZ = angles.thirdAngle;
        
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        
        dangles[0] = ((dangles[0] * damping) + angles.firstAngle) / (damping + 1);
        dangles[1] = ((dangles[1] * damping) + angles.secondAngle) / (damping + 1);
        dangles[2] = ((dangles[2] * damping) + angles.thirdAngle) / (damping + 1);
        
        gravity = imu.getGravity();
    }


    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    /*public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }*/
}
