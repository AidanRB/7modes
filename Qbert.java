package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

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
    public DcMotor arm = null;
    public DcMotor slide = null;
    public CRServo grab = null;
    public CRServo hand = null;
    public Servo mark = null;
    public BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    public float dangles[] = new float[3];

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    //private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Qbert(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        one   = ahwMap.dcMotor.get("one");
        two  = ahwMap.dcMotor.get("two");
        three    = ahwMap.dcMotor.get("three");
        four    = ahwMap.dcMotor.get("four");
        lift = ahwMap.dcMotor.get("lift");
        arm = ahwMap.dcMotor.get("arm");
        slide = ahwMap.dcMotor.get("slide");
        grab = ahwMap.crservo.get("grab");
        hand = ahwMap.crservo.get("hand");
        mark = ahwMap.servo.get("mark");

        // Set all motors to zero power
        /*one.setPower(0);
        two.setPower(0);
        three.setPower(0);
        four.setPower(0);*/

        // Set all motors to run using encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        three.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        four.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        three.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        four.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        
        imu = ahwMap.get(BNO055IMU.class, "imu");
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
