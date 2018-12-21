package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is NOT an opmode.
 *
 * This class is the hardware map of the robot.
 */
public class OneRobotToRuleThemAll{
    /* Public OpMode members. */
    public DcMotor aragorn = null;
    public DcMotor legolas = null;
    public DcMotor gimli = null;
    public DcMotor boromir = null;
    public BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    public float dangles[] = new float[3];

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    //private ElapsedTime period  = new ElapsedTime();

    /* Constructor */  
    public OneRobotToRuleThemAll(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        aragorn   = ahwMap.dcMotor.get("aragorn");
        legolas  = ahwMap.dcMotor.get("legolas");
        gimli    = ahwMap.dcMotor.get("gimli");
        boromir   = ahwMap.dcMotor.get("boromir");

        gimli.setDirection(DcMotor.Direction.REVERSE);
        boromir.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        /*frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);*/

        // Set all motors to run using encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        /*frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        aragorn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        legolas.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gimli.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        boromir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
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
