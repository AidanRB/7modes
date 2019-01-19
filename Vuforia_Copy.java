package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name="AutoVLeft", group="Concept")

public class Vuforia_Copy extends LinearOpMode {
    private static final String VUFORIA_KEY = "AaIK0f//////AAAAGTsgmHszM030skFBvcnAJlNSWaH7oKLZhxAZeCi7ToBGSKkO7T3EvzsRVYQdyDp2X+TFK6TQs+3WoCHkZXDYPQd87f77D6kvcBr8zbJ07Fb31UKiXdUBvX+ZQSV3kBhdAoxhfMa0WPgys7DYaeiOmM49CsNra7nVh05ls0th3h07wwHz3s/PBZnQwpbfr260CDgqBv4e9D79Wg5Ja5p+HAOJvyqg2r/Z5dOyRvVI3f/jPBRZHvDgDF9KTcuJAPoDHxfewmGFOFtiUamRLvcrkK9rw2Vygi7w23HYlzFO7yap+jUk1bv0uWNc0j5HPJDAjqa2ijBN9aVDrxzmFJml5WMA3GJJp8WOd9gkGhtI/BIo";
    private Qbert robot = new Qbert();

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        robot.init(hardwareMap);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT  = -110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 170;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 100;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();
        
                // run until the end of the match (driver presses STOP)
        robot.lift.setTargetPosition(robot.lift.getCurrentPosition() - 32000);
        robot.lift.setPower(100);
        while(robot.lift.isBusy()){}
        //robot.lift.setPower(0);
        
        robot.grab.setPower(-1);
        pause(2000);
        robot.grab.setPower(0);
        telemetry.addData("lift pos", robot.lift.getCurrentPosition());
        telemetry.update();
        //pause(5000);
        robot.lift.setTargetPosition(robot.lift.getCurrentPosition() + 32000);
        
        driveBackward(0.8, 17); // away from lander
        pause(200);
        driveRight(0.8, 18); // to side of field
        pause(200);
        driveBackward(0.8, 8); // knock off cube/whatever
        //pause(200);
        //driveRight(0.7, 18); // to side of field
        //pause(200);
        //turnCounter(0.5, 42); // to ???
        turnEnc(-50, 9);                                              /////////////
        pause(200);
        driveForward(0.8, 18); // to middle
        pause(200);
        driveRight(0.8, 20);
        pause(200);
        driveForward(0.8, 55);
        turnEnc(50, 18);                                        ///////
        robot.hand.setPower(-1);
        pause(2000);
        robot.hand.setPower(0);
        pause(200);
        driveLeft(0.8, 75);
        robot.arm.setPower(-0.2);
        pause(3000);
        robot.arm.setPower(0);
        /*robot.hand.setPower(-1); // drop marker
        pause(2000);
        robot.hand.setPower(0);
        driveLeft(0.7, 70); // */
        
        //while(robot.lift.isBusy()) {}
        
        /*while (opModeIsActive()) {
            // check all the trackable target to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                if(Math.abs(rotation.thirdAngle) > 10000) {
                    robot.one.setPower(rotation.thirdAngle/1000);
                    robot.two.setPower(rotation.thirdAngle/1000);
                    robot.three.setPower(rotation.thirdAngle/1000);
                    robot.four.setPower(rotation.thirdAngle/1000);
                }
            }
            else {
                telemetry.addData("Visible Target", "none");
                stopWheels();
            }
            telemetry.update();
            
        }*/
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

    private void turnEnc(double speed, double distance) {
        robot.one.setPower(speed);
        robot.two.setPower(speed);
        robot.three.setPower(speed);
        robot.four.setPower(speed);
        
        int initial = robot.one.getCurrentPosition();
        if(speed > 0) {
            while(robot.one.getCurrentPosition() < initial + distance*1075.2/3.1415/5) {
                telemetry.addData("one", robot.one.getCurrentPosition());
                telemetry.update();
            }
        } else {
            while(robot.one.getCurrentPosition() > initial - distance*1075.2/3.1415/5) {
                telemetry.addData("one", robot.one.getCurrentPosition());
                telemetry.update();
            }
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
