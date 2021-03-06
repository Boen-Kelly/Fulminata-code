package org.firstinspires.ftc.teamcode;

import android.drm.DrmErrorEvent;
import android.icu.util.RangeValueIterator;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_232H;
import org.firstinspires.ftc.teamcode.Blue_autonomous.Blue_side_tray_block_back;
import org.firstinspires.ftc.teamcode.vuforia_test;

import java.awt.font.NumericShaper;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
import static org.firstinspires.ftc.teamcode.gearedup_vuforia.closeEnoughRot;
import static org.firstinspires.ftc.teamcode.gearedup_vuforia.closeEnoughX;
import static org.firstinspires.ftc.teamcode.gearedup_vuforia.closeEnoughY;
import static org.firstinspires.ftc.teamcode.gearedup_vuforia.correctionRot;
import static org.firstinspires.ftc.teamcode.gearedup_vuforia.correctionX;
import static org.firstinspires.ftc.teamcode.gearedup_vuforia.correctionY;

@Autonomous
//@Disabled
public class vuforiaAutonomous extends LinearOpMode {

    private DcMotorEx backLeftWheel, backRightWheel, frontLeftWheel, frontRightWheel;
    private DcMotor linearLift,  linearLift2;
    private Servo CLAW, trayServoL, trayServoR;
    BNO055IMU imu;
    Orientation angles;
    DistanceSensor Distance;
    DigitalChannel Touch;
    private ElapsedTime runtime = new ElapsedTime();
    OpenGLMatrix robotLocationTransform;
    VuforiaTrackableDefaultListener listener;
    VectorF trans;
    Orientation rot;
    boolean targetNotSeen = true;



    /**
     * This code (below) sets the parameters for vuforia blocks
     */
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    private static final boolean PHONE_IS_PORTRAIT = true;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            " ARPBgCz/////AAABmRcdPObheUUygHmBUcN8C1JoEljLHy24QOHJDCHW4Gq6uDeqnxXUcCydym407A7o72Uk0O43Cd06XCYAVUb48quH7aKTXgI+i27SIB5f5JJWJ2t4J7uRs7HhXienQXmqENzQm8UcZnhjc2nGQXm7a3ArXAhfUpGswuwofGmioy95bOOMsIG//52TTv+UaTJsNuFN7+V+7m3Bz/YbmfGQVUvyA5/ouFdkoqMBEu3SM8oPFXyK1LFHNbjaMm5+z8QRkf/y0/7xCL0osTWYFedIjADfj4hzx5i5qR32Pk2LzgThG3Q+oXljHQluSSBA2nwvb7vwfz+ICG2LTb7+JpWBaMHAX6ikcQf/eIyet1b+YjMg  ";
    public static String positionSkystone = "";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;
    private VuforiaBase targetsSkyStone;
    private Iterable<? extends VuforiaTrackable> allTrackables;
    double rampPower = 0;
    boolean closeEnough = false;
    double actualTransX;
    double actualTransY;
    double actualRot;
    boolean left = true;
    double targetHeading;

    int[] rotationArray = new int[10];



    /**
     * This code ^ sets the parameters for vuforia blocks
     */
    @Override
    public void runOpMode() throws InterruptedException {

        rotationArray[9] = 5;
        rotationArray[0] = -5;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        backLeftWheel = hardwareMap.get(DcMotorEx.class, "Back_left_wheel");
        backRightWheel = hardwareMap.get(DcMotorEx.class, "Back_right_wheel");
        frontLeftWheel = hardwareMap.get(DcMotorEx.class, "Front_left_wheel");
        frontRightWheel = hardwareMap.get(DcMotorEx.class, "Front_right_wheel");
        CLAW = hardwareMap.servo.get("CLAW");
        linearLift = hardwareMap.get(DcMotor.class, " linearLift");
        linearLift2 = hardwareMap.get(DcMotor.class, " linearLift2");
        trayServoL = hardwareMap.get(Servo.class, "trayServoL");
        trayServoR = hardwareMap.get(Servo.class, "trayServoR");
        Distance = hardwareMap.get(DistanceSensor.class, "Distance");
        Touch = hardwareMap.get(DigitalChannel.class, "Touch");
        imu.initialize(parameters);

        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        //frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        //linearLift.setDirection(DcMotorSimple.Direction.REVERSE);
        //linearLift2.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Touch.setMode(DigitalChannel.Mode.INPUT);

        trayServoR.setPosition(1);
        trayServoL.setPosition(0);

        CLAW.setPosition(.65);

        while ((!imu.isGyroCalibrated()) && !isStopRequested()) {

        }

        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeftWheel.setVelocityPIDFCoefficients(75,25,10,0);
        backRightWheel.setVelocityPIDFCoefficients(75,25,10,0);
        frontLeftWheel.setVelocityPIDFCoefficients(75,25,10,0);
        frontRightWheel.setVelocityPIDFCoefficients(75,25,10,0);



        //This code ^ virtually tells the vuforia program where everything is


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        vuforiaparameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforiaparameters.cameraDirection = CAMERA_CHOICE;
        vuforiaparameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaparameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

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
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneXRotate = 90;
        } else {
            phoneXRotate = -90;
        }

        // Rotate the phone vertical about the Y axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneYRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 180;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 5.93f * mmPerInch;   // eg: Camera is 5.93 Inches above ground when portrait, and 7.6875 when landscape
        final float CAMERA_LEFT_DISPLACEMENT = -57;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, vuforiaparameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        //This code ^ virtually tells the vuforia program where everything is

        CameraDevice.getInstance().setFlashTorchMode(true);

        targetsSkyStone.activate();

        telemetry.addData("Status:", "initialized");
        telemetry.update();

        //waitForStart();



        //CameraDevice.getInstance().setFlashTorchMode(true);

        waitForStart();


        //Drive(410,.7,0,0);

        //DrivewOutRamp(140,0,.4,0);
        Drive(250,.6,.1,0);

        sleep(250);

        //Drive(165,0,.4,0);

        //sleep(230);

        int startPosition = backLeftWheel.getCurrentPosition();

         double backLPower = 0;
         double backRPower = 0;
         double frontLPower = 0;
         double frontRPower = 0;

         double startTime = runtime.time();

        //backLeftWheel.setVelocity(60,angleUnit.DEGREES);




        while (!isStopRequested() && !targetVisible && ((backLeftWheel.getCurrentPosition() > (-100 + startPosition)) && (backRightWheel.getCurrentPosition() > (-100 + startPosition)))) {

            /**targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }*/

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

                VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) stoneTarget.getListener();
                OpenGLMatrix location;
                VectorF translation = lastLocation.getTranslation();

                if ((stoneTarget != null) && (listener != null) && listener.isVisible()){
                    location = listener.getUpdatedRobotLocation();
                    if (location != null){
                        VectorF trans = location.getTranslation();
                        Orientation rot = Orientation.getOrientation(location, EXTRINSIC, XYZ, DEGREES);

                        telemetry.addData("y translation:", trans.get(1));
                    }
                }
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                targetNotSeen = false;
                sleep(250);
                break;
            }else {
                telemetry.addData("Visible Target", "none");

                double backleftSpeed, backrightSpeed, frontleftSpeed, frontrightSpeed;


                backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                 //startPosition = backLeftWheel.getCurrentPosition();
                 frontRPower = 0 - (-.135);
                 frontLPower = 0 + (-.135);
                 backRPower = 0 + (-.135);
                 backLPower = 0 - (-.135);
                boolean rampUp = true;
                boolean rampDown = false;
                //while (((backLeftWheel.getCurrentPosition() < (distance + startPosition)) && (backRightWheel.getCurrentPosition() < (distance + startPosition))) && !isStopRequested()) {

                if((startPosition < backLeftWheel.getCurrentPosition()) &&(backLeftWheel.getCurrentPosition() < startPosition + 70)){
                        rampUp = true;
                        rampDown = false;
                    }


                    if (rampUp == true){
                        rampPower += .15;
                        //sleep(5);

                        if (rampPower >= 1){
                            rampUp = false;
                            rampPower = 1;
                        }
                    } else if (rampDown) {
                        rampPower -= .15;
                        //sleep(7);
                        if (rampPower <= .3){
                            rampDown = ! rampDown;
                            rampPower = .3;
                        }
                    } else{
                        rampPower = 1;
                    }



                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    double currentHeading = angles.firstAngle;

                    double correction = (0 - currentHeading) / 100;

                    backleftSpeed = backLPower - (correction);
                    backrightSpeed = backRPower + (correction);
                    frontleftSpeed = frontLPower - (correction);
                    frontrightSpeed = frontRPower + (correction);

                    backrightSpeed = Range.clip(backrightSpeed, -1, 1);
                    backleftSpeed = Range.clip(backleftSpeed, -1, 1);
                    frontleftSpeed = Range.clip(frontleftSpeed, -1, 1);
                    frontrightSpeed = Range.clip(frontrightSpeed, -1, 1);


                    backLeftWheel.setVelocity(((backleftSpeed * rampPower) * 540),DEGREES);
                    frontLeftWheel.setVelocity(((frontleftSpeed * rampPower) * 540),DEGREES);
                    backRightWheel.setVelocity(((backrightSpeed * rampPower) * 540),DEGREES);
                    frontRightWheel.setVelocity(((frontrightSpeed * rampPower) * 540), DEGREES);


                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("current heading:", angles.firstAngle);
                    telemetry.addData("desired heading:", startPosition);
                    telemetry.addData("motor speed:", backLeftWheel.getPower());


            }
            telemetry.update();
        }

        backLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontLeftWheel.setPower(0);
        frontRightWheel.setPower(0);

        int endPosition = backLeftWheel.getCurrentPosition();

        if (((startPosition - 30) <= endPosition) && (endPosition <= (startPosition + 30))){
            positionSkystone = "right";
        } else if (((startPosition + 50) <= endPosition) && (endPosition <= (startPosition + 110))){
            positionSkystone = "center";
        } else if ((startPosition + 160) <= endPosition){
            positionSkystone = "left";
        } else{
            positionSkystone = "left";
        }

        telemetry.addData("skystone position", positionSkystone);
        telemetry.update();
        //sleep(2000);

        //Drive(270,.7,0,0);





        if (!targetNotSeen) {
            //**
            if (((actualTransY == 0) && (actualTransX == 0)) && (actualRot == 0)){
                targetsSkyStone.deactivate();
                targetsSkyStone.activate();
                telemetry.addLine("capture error");
                telemetry.update();
                sleep(250);
            }
                //*/

                while ((!((closeEnoughX && closeEnoughY) && closeEnoughRot)) && !isStopRequested()) {

                    targetVisible = false;
                    for (VuforiaTrackable trackable : allTrackables) {
                        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                            telemetry.addData("Visible Target", trackable.getName());
                            targetVisible = true;

                            // getUpdatedRobotLocation() will return null if no new information is available since
                            // the last time that call was made, or if the trackable is not currently visible.
                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                            if (robotLocationTransform != null) {
                                lastLocation = robotLocationTransform;
                            }
                            break;
                        }
                    }

                    // Provide feedback as to where the robot is located (if we know).
                    if ((targetVisible && !((closeEnoughY && closeEnoughX) && closeEnoughRot)) && !isStopRequested()) {
                        // express position (translation) of robot in inches.

                        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) stoneTarget.getListener();
                        listener.addTrackable(stoneTarget);
                        //VuforiaTrackable listining = stoneTarget;
                        //listining.getListener();
                        OpenGLMatrix location;
                        VectorF translation = lastLocation.getTranslation();

                        if ((stoneTarget != null) && (listener != null) && listener.isVisible()) {
                            location = listener.getRobotLocation();
                            if (location != null) {
                                trans = location.getTranslation();
                                rot = Orientation.getOrientation(location, EXTRINSIC, XYZ, DEGREES);
                                //telemetry.update();
                            }
                        }
                        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                        // express the rotation of the robot in degrees.
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                        telemetry.addData("y translation:", trans.get(1));

                        actualTransY = trans.get(1);
                        actualTransX = trans.get(0);
                        actualRot = -rot.thirdAngle;


                        if ((-490 <= actualTransX) && (actualTransX <= 0)) {
                            closeEnoughX = true;
                        } else {
                            closeEnoughX = false;
                        }

                        if ((-3 <= actualTransY) && (actualTransY <= 3)) {
                            closeEnoughY = true;
                        } else {
                            closeEnoughY = false;
                        }

                        if ((-3 <= actualRot) && (actualRot <= 3)) {
                            closeEnoughRot = true;
                        } else {
                            closeEnoughRot = false;
                        }

                        if (closeEnoughRot) {
                            correctionRot = 0;
                        } else {
                            if ((-2 <= actualRot) && (actualRot <= 2)) {
                                correctionRot = (0 - actualRot) * 1.4;
                            } else {
                                correctionRot = (0 - actualRot) / 150;
                            }
                        }

                        if (closeEnoughX) {
                            correctionX = 0;
                        } else {
                            correctionX = (490 - actualTransX) / 2000;
                        }

                        if (closeEnoughY) {
                            correctionY = 0;
                        } else {
                            if ((-3 <= actualTransY) && (actualTransY <= 3)) {
                                correctionY = (0 - actualTransY) / 125;
                            } else {
                                correctionY = (0 - actualTransY) / 300;
                            }
                        }


                        /**if (((0 <= correctionX) && (correctionX <= .2)) && !closeEnoughX) {
                         correctionX = .2;
                         }

                         if (((-.2 <= correctionX) && (correctionX <= 0)) && !closeEnoughX) {
                         correctionX = -.2;
                         }

                         if (((0 <= correctionY) && (correctionY <= .6)) && !closeEnoughY) {
                         correctionY = .6;
                         }

                         if (((-.6 <= correctionY) && (correctionY <= 0)) && !closeEnoughY) {
                         correctionY = -.6;
                         }

                         if (((0 <= correctionRot) && (correctionRot <= .4)) && !closeEnoughRot) {
                         correctionRot = .4;
                         }

                         if (((-.4 <= correctionRot) && (correctionRot <= 0)) && !closeEnoughRot) {
                         correctionRot = -.4;
                         }
                         //*/
                        double frontRightPower = -correctionRot + correctionY + correctionX;
                        double frontLeftPower = correctionRot - correctionY + correctionX;
                        double backRightPower = -correctionRot - correctionY + correctionX;
                        double backLeftPower = correctionRot + correctionY + correctionX;

                        frontRPower = Range.clip(frontRPower, -1, 1);
                        frontLPower = Range.clip(frontLPower, -1, 1);
                        backRPower = Range.clip(backRPower, -1, 1);
                        backLPower = Range.clip(backLPower, -1, 1);

                        backLeftWheel.setVelocity(((backLeftPower) * 540), DEGREES);
                        backRightWheel.setVelocity(((backRightPower) * 540), DEGREES);
                        frontLeftWheel.setVelocity(((frontLeftPower) * 540), DEGREES);
                        frontRightWheel.setVelocity(((frontRightPower) * 540), DEGREES);
                        telemetry.addData("correction y rate:", correctionY);
                        telemetry.addData("correction x rate:", correctionX);
                        telemetry.addData("corection rotation rate:", correctionRot);
                        telemetry.addData("x axis actual:", actualTransX);
                        telemetry.addData("y axis actual:", actualTransY);
                        telemetry.addData("closeEnoughY", closeEnoughY);
                        telemetry.addData("closeEnoughX", closeEnoughX);
                        telemetry.addData("closeEnoughRot", closeEnoughRot);


                    } else {
                        telemetry.addData("Visible Target", "none");
                        /**if (left){
                         turnLeft(45);
                         } else{
                         turnRight(-45);
                         }

                         left = !left;*/

                        backLeftWheel.setPower(0);
                        frontLeftWheel.setPower(0);
                        backRightWheel.setPower(0);
                        frontRightWheel.setPower(0);
                    }
                    telemetry.update();




                //sleep(5000);
            }
        }

        backLeftWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontRightWheel.setPower(0);

        //drivewVuforia(stoneTarget,0,-.3,1120,0);



        telemetry.addData("Skystone position:", positionSkystone);
        telemetry.addData("wheel movement:", endPosition);
        telemetry.addData("start position:",startPosition);
        telemetry.addData("correction y rate:", correctionY);
        telemetry.addData("correction x rate:", correctionX);
        telemetry.addData("corection rotation rate:", correctionRot);
        telemetry.addData("x axis actual:", actualTransX);
        telemetry.addData("y axis actual:", actualTransY);
        telemetry.addData("actual rotation:", actualRot);
        telemetry.addData("closeEnoughY", closeEnoughY);
        telemetry.addData("closeEnoughX", closeEnoughX);
        telemetry.addData("closeEnoughRot", closeEnoughRot);
        telemetry.update();
        //sleep(200000);
        targetsSkyStone.deactivate();

        //CameraDevice.getInstance().setFlashTorchMode(false);

        if (positionSkystone.equals("right")){
            //DrivewOutRamp(90,0,.5,0);
            Lift(1780,1);
            CLAW.setPosition(0);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            targetHeading = angles.firstAngle;
            DrivewDistance(4,.3,0,targetHeading);
            Drop(0,1);
            CLAW.setPosition(1);
            sleep(500);
            Lift(270,1);
            backupwOutRamp(-100,-.5,0,0);
            //backup(-200,0,-.7,0);
            backup(-1080,0,-1,7);
            //backup(-200,0,-.7,0);
        } else if (positionSkystone.equals("center")){
            Lift(1780,1);
            CLAW.setPosition(0);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            targetHeading = angles.firstAngle;
            DrivewDistance(4,.3,0,targetHeading);
            Drop(0,1);
            CLAW.setPosition(1);
            sleep(500);
            Lift(270,1);
            backupwOutRamp(-100,-.5,0,0);
            //backup(-200,0,-.7,90);
            backup(-950,0,-1,7);
            //backup(-200,0,-.7,90);
        }else if (positionSkystone.equals("left")){
            Lift(1780,1);
            CLAW.setPosition(0);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            targetHeading = angles.firstAngle;
            DrivewDistance(4,.3,0,targetHeading);
            Drop(0,1);
            CLAW.setPosition(1);
            sleep(500);
            Lift(270,1);
            backupwOutRamp(-100,-.5,0,0);
            //backup(-200,0,-.7,90);
            backup(-900,0,-1,7);
            //backup(-200,0,-.7,90);
        }

        Lift(1970,1);

        DrivewDistance(3,.4,0,0);
        DrivewOutRamp(20,.4,0,0);

        trayServoL.setPosition(1);
        trayServoR.setPosition(0);
        sleep(500);

        backup(-500,-3,0,70);


        turnLeft(100);

        startPosition = backLeftWheel.getCurrentPosition();

        while (backLeftWheel.getCurrentPosition() < (startPosition + 140)){
            backLeftWheel.setVelocity(540,DEGREES);
            backRightWheel.setVelocity(540,DEGREES);
            frontLeftWheel.setVelocity(540,DEGREES);
            frontRightWheel.setVelocity(540,DEGREES);
            CLAW.setPosition(0);
        }

        trayServoL.setPosition(0);
        trayServoR.setPosition(1);

        CLAW.setPosition(0);



        backup(-240, -1,0,90);

        Drop(0,1);

        backup(-370,-1,0,100);

        CLAW.setPosition(1);

        sleep(1000);

        /**
        telemetry.addData("time:", runtime.time());
        telemetry.update();
        sleep(999999);
         //*/


    }

    public void Drive ( int distance, double straight, double strafe, int target) throws InterruptedException {

        double backleftSpeed, backrightSpeed, frontleftSpeed, frontrightSpeed;


        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double startPosition = backLeftWheel.getCurrentPosition();
        double frontRPower = straight - strafe;
        double frontLPower = straight + strafe;
        double backRPower = straight + strafe;
        double backLPower = straight - strafe;
        boolean rampUp = true;
        boolean rampDown = false;
        while (((backLeftWheel.getCurrentPosition() < (distance + startPosition)) && (backRightWheel.getCurrentPosition() < (distance + startPosition))) && !isStopRequested()) {
            if((startPosition < backLeftWheel.getCurrentPosition()) &&(backLeftWheel.getCurrentPosition() < startPosition + 70)){
                rampUp = true;
                rampDown = false;
            }else if ((((distance + startPosition) - 160) < backLeftWheel.getCurrentPosition()) && (backLeftWheel.getCurrentPosition() < (distance + startPosition))){
                rampDown = true;
                rampUp = false;
            }


            if (rampUp == true){
                rampPower += .15;
                sleep(5);

                if (rampPower >= 1){
                    rampUp = false;
                    rampPower = 1;
                }
            } else if (rampDown) {
                rampPower -= .15;
                sleep(3);
                if (rampPower <= .3){
                    rampDown = ! rampDown;
                    rampPower = .3;
                }
            } else{
                rampPower = 1;
            }



            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

            double correction = (target - currentHeading) / 100;

            backleftSpeed = backLPower - (correction);
            backrightSpeed = backRPower + (correction);
            frontleftSpeed = frontLPower - (correction);
            frontrightSpeed = frontRPower + (correction);

            backrightSpeed = Range.clip(backrightSpeed, -1, 1);
            backleftSpeed = Range.clip(backleftSpeed, -1, 1);
            frontleftSpeed = Range.clip(frontleftSpeed, -1, 1);
            frontrightSpeed = Range.clip(frontrightSpeed, -1, 1);


            backLeftWheel.setVelocity(((backleftSpeed * rampPower) * 540),DEGREES);
            frontLeftWheel.setVelocity(((frontleftSpeed * rampPower) * 540),DEGREES);
            backRightWheel.setVelocity(((backrightSpeed * rampPower) * 540),DEGREES);
            frontRightWheel.setVelocity(((frontrightSpeed * rampPower) * 540), DEGREES);


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("current heading:", angles.firstAngle);
            telemetry.addData("desired heading:", startPosition);
            telemetry.addData("motor speed:", backLeftWheel.getPower());
        }

        backLeftWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontRightWheel.setPower(0);


    }

    //**
     public void Lift (int Height, double power)throws InterruptedException{
        while((linearLift.getCurrentPosition() >= -Height) && !isStopRequested()){
            linearLift.setPower(-power);
            linearLift2.setPower(-power);
        }
        linearLift.setPower(0);
        linearLift2.setPower(0);
    }

    public void Drop (int Height, double power)throws InterruptedException{
        while((linearLift.getCurrentPosition() <= -Height) && !isStopRequested()){
            linearLift.setPower(power);
            linearLift2.setPower(power);
        }
        linearLift.setPower(0);
        linearLift2.setPower(0);
    }
     //*/

    public void backup(int distance, double straight, double strafe, int target) throws InterruptedException {


        double backleftSpeed, backrightSpeed, frontleftSpeed, frontrightSpeed;


        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double startPosition = backLeftWheel.getCurrentPosition();
        double frontRPower = straight - strafe;
        double frontLPower = straight + strafe;
        double backRPower = straight + strafe;
        double backLPower = straight - strafe;
        boolean rampUp = true;
        boolean rampDown = false;
        while (((backLeftWheel.getCurrentPosition() > (distance + startPosition)) && (backRightWheel.getCurrentPosition() > (distance + startPosition))) && !isStopRequested()) {
            if((startPosition > backLeftWheel.getCurrentPosition()) &&(backLeftWheel.getCurrentPosition() > startPosition - 70)){
                rampUp = true;
                rampDown = false;
            }else if ((((distance + startPosition) + 180) > backLeftWheel.getCurrentPosition()) && (backLeftWheel.getCurrentPosition() > (distance + startPosition))){
                rampDown = true;
                rampUp = false;
            }


            if (rampUp == true){
                rampPower += .15;
                sleep(5);

                if (rampPower >= 1){
                    rampUp = false;
                    rampPower = 1;
                }
            } else if (rampDown) {
                rampPower -= .15;
                sleep(5);
                if (rampPower <= .3){
                    rampDown = ! rampDown;
                    rampPower = .3;
                }
            } else{
                rampPower = 1;
            }



            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

            double correction = (target - currentHeading) / 100;

            backleftSpeed = backLPower - (correction * 5);
            backrightSpeed = backRPower + (correction * 5);
            frontleftSpeed = frontLPower - (correction * 5);
            frontrightSpeed = frontRPower + (correction * 5);

            backrightSpeed = Range.clip(backrightSpeed, -1, 1);
            backleftSpeed = Range.clip(backleftSpeed, -1, 1);
            frontleftSpeed = Range.clip(frontleftSpeed, -1, 1);
            frontrightSpeed = Range.clip(frontrightSpeed, -1, 1);


            backLeftWheel.setVelocity(((backleftSpeed * rampPower) * 540),DEGREES);
            frontLeftWheel.setVelocity(((frontleftSpeed * rampPower) * 540),DEGREES);
            backRightWheel.setVelocity(((backrightSpeed  * rampPower) * 540),DEGREES);
            frontRightWheel.setVelocity(((frontrightSpeed * rampPower) * 540),DEGREES);


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("current heading:", angles.firstAngle);
            telemetry.addData("desired heading:", startPosition);
            telemetry.addData("motor speed:", backLeftWheel.getPower());
        }

        backLeftWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontRightWheel.setPower(0);



    }

    public void DrivewDistance(double range, double straight, double strafe, double target) throws InterruptedException {

        double backleftSpeed, backrightSpeed, frontleftSpeed, frontrightSpeed;


        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startPosition = backLeftWheel.getCurrentPosition();
        double frontRPower = straight - strafe;
        double frontLPower = straight + strafe;
        double backRPower = straight + strafe;
        double backLPower = straight - strafe;

        while (!(Distance.getDistance(DistanceUnit.INCH) < range) && !isStopRequested()) {


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

            double correction = (target - currentHeading) / 100;

            backleftSpeed = backLPower - (correction * 2);
            backrightSpeed = backRPower + (correction * 2);
            frontleftSpeed = frontLPower - (correction * 2);
            frontrightSpeed = frontRPower + (correction * 2);

            backrightSpeed = Range.clip(backrightSpeed, -1, 1);
            backleftSpeed = Range.clip(backleftSpeed, -1, 1);
            frontleftSpeed = Range.clip(frontleftSpeed, -1, 1);
            frontrightSpeed = Range.clip(frontrightSpeed, -1, 1);


            backLeftWheel.setVelocity(((backleftSpeed) * 540),DEGREES);
            frontLeftWheel.setVelocity(((frontleftSpeed) * 540),DEGREES);
            backRightWheel.setVelocity(((backrightSpeed) * 540),DEGREES);
            frontRightWheel.setVelocity(((frontrightSpeed) * 540),DEGREES);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("current heading:", angles.firstAngle);
            telemetry.addData("desired heading:", startPosition);
        }

        backLeftWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontRightWheel.setPower(0);


    }

    public void turnRight(double desiredHeading) {

        double speed;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startAngle = angles.firstAngle;


        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while ((Math.abs(angles.firstAngle) <= (desiredHeading - 2)) && !isStopRequested()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            double remainingAngle = -desiredHeading - angles.firstAngle;
            double dividingRatio = -desiredHeading - startAngle;

            speed = remainingAngle / dividingRatio;


            if (Math.abs(speed) < .2) {
                speed = .2;
            } else {
                speed = remainingAngle / dividingRatio;
            }

            backLeftWheel.setPower(speed);
            frontLeftWheel.setPower(speed);
            backRightWheel.setPower(-speed);
            frontRightWheel.setPower(-speed);


        }
        backLeftWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontRightWheel.setPower(0);

    }

    public void turnLeft(double desiredHeading) {

        double speed;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startAngle = angles.firstAngle;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while ((angles.firstAngle <= (desiredHeading - 2)) && !isStopRequested()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double remainingAngle = desiredHeading - angles.firstAngle;
            double dividingRatio = desiredHeading - startAngle;

            speed = remainingAngle / dividingRatio;

            if (Math.abs(speed) < .2) {
                speed = .2;
            } else {
                speed = remainingAngle / dividingRatio;
            }

            backLeftWheel.setPower(-speed);
            frontLeftWheel.setPower(-speed);
            backRightWheel.setPower(speed);
            frontRightWheel.setPower(speed);

        }
        backLeftWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontRightWheel.setPower(0);

    }

    public void DrivewTouch(double straight, double strafe, int target, double seconds) throws InterruptedException {

        double backleftSpeed, backrightSpeed, frontleftSpeed, frontrightSpeed;


        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startPosition = backLeftWheel.getCurrentPosition();
        double frontRPower = straight - strafe;
        double frontLPower = straight + strafe;
        double backRPower = straight + strafe;
        double backLPower = straight - strafe;
        double timeLimit = runtime.time() + seconds;

        while (((Touch.getState() == true) && !isStopRequested()) && (runtime.time() < timeLimit)) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

            double correction = (target - currentHeading) / 100;

            backleftSpeed = backLPower - (correction * 2);
            backrightSpeed = backRPower + (correction * 2);
            frontleftSpeed = frontLPower - (correction * 2);
            frontrightSpeed = frontRPower + (correction * 2);

            backrightSpeed = Range.clip(backrightSpeed, -1, 1);
            backleftSpeed = Range.clip(backleftSpeed, -1, 1);
            frontleftSpeed = Range.clip(frontleftSpeed, -1, 1);
            frontrightSpeed = Range.clip(frontrightSpeed, -1, 1);


            backLeftWheel.setPower(backleftSpeed);
            frontLeftWheel.setPower(frontleftSpeed);
            backRightWheel.setPower(backrightSpeed);
            frontRightWheel.setPower(frontrightSpeed);


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("current heading:", angles.firstAngle);
            telemetry.addData("desired heading:", startPosition);
        }

        backLeftWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontRightWheel.setPower(0);


    }

    /**public void trackskystone() throws InterruptedException {

        while (!isStopRequested()){ //&& (((positionSkystone != "left") && positionSkystone != "center") && positionSkystone != "right")) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    if (trackable.getName().equals("Stone Target")) {
                        telemetry.addLine("Stone target is visible");
                    }
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
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

                double yPosition = translation.get(1);
                if (yPosition > -7) {
                    positionSkystone = "left";
                } else if (yPosition > -10) {
                    positionSkystone = "center";
                } else if (yPosition < -10) {
                    positionSkystone = "right";
                } else {
                    positionSkystone = "Nothing";
                }

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }else {

            }
            telemetry.addData("Skystone Position", positionSkystone);
            telemetry.update();

            /**if (positionSkystone == "left") {
             break;
             } else if (positionSkystone == "center") {
             break;
             } else if (positionSkystone == "right") {
             break;
             }
        }
    }*/

    public void drivewVuforia (VuforiaTrackable Target,double straight,double strafe,int distance, int targetHeading)throws InterruptedException{
        int startPosition = backLeftWheel.getCurrentPosition();
        double backleftSpeed, backrightSpeed, frontleftSpeed, frontrightSpeed;


        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //double startPosition = backLeftWheel.getCurrentPosition();
        double frontRPower = straight - strafe;
        double frontLPower = straight + strafe;
        double backRPower = straight + strafe;
        double backLPower = straight - strafe;
        boolean rampUp = true;
        boolean rampDown = false;

        telemetry.addLine("initialized");
        telemetry.update();
        sleep(5000);

        while (!isStopRequested() && ((backLeftWheel.getCurrentPosition() < (distance + startPosition)) && (backRightWheel.getCurrentPosition() < (distance + startPosition))) ) {

            if((startPosition < backLeftWheel.getCurrentPosition()) &&(backLeftWheel.getCurrentPosition() < startPosition + 70)){
                rampUp = true;
                rampDown = false;
            }else if ((((distance + startPosition) - 210) < backLeftWheel.getCurrentPosition()) && (backLeftWheel.getCurrentPosition() < (distance + startPosition))){
                rampDown = true;
                rampUp = false;
            }


            if (rampUp == true){
                rampPower += .15;
                sleep(5);

                if (rampPower >= 1){
                    rampUp = false;
                    rampPower = 1;
                }
            } else if (rampDown) {
                rampPower -= .15;
                sleep(5);
                if (rampPower <= .3){
                    rampDown = ! rampDown;
                    rampPower = .3;
                }
            } else{
                rampPower = 1;
            }

            telemetry.addLine("after ramp up");
            telemetry.update();
            sleep(5000);

            // check all the trackable targets to see which one (if any) is visible.
            /**targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    if (trackable.getName().equals("Stone Target")) {
                        telemetry.addLine("Stone target is visible");
                    }
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.

                    //VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
                    listener = (VuforiaTrackableDefaultListener) Target.getListener();
                    robotLocationTransform = listener.getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }*/
            /**targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                telemetry.addLine("is target being seen?");
                telemetry.update();
                sleep(5000);
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
            }*/

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                telemetry.addLine("target seen");
                telemetry.update();
                sleep(5000);
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                break;
            }else {
                telemetry.addLine("regular drive");
                telemetry.update();
                sleep(5000);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double currentHeading = angles.firstAngle;

                double correction = (targetHeading - currentHeading) / 100;

                backleftSpeed = backLPower - (correction * 2);
                backrightSpeed = backRPower + (correction * 2);
                frontleftSpeed = frontLPower - (correction * 2);
                frontrightSpeed = frontRPower + (correction * 2);

                backrightSpeed = Range.clip(backrightSpeed, -1, 1);
                backleftSpeed = Range.clip(backleftSpeed, -1, 1);
                frontleftSpeed = Range.clip(frontleftSpeed, -1, 1);
                frontrightSpeed = Range.clip(frontrightSpeed, -1, 1);


                backLeftWheel.setPower(backleftSpeed * rampPower);
                frontLeftWheel.setPower(frontleftSpeed * rampPower);
                backRightWheel.setPower(backrightSpeed * rampPower);
                frontRightWheel.setPower(frontrightSpeed * rampPower);

            }
            telemetry.addData("Skystone Position", positionSkystone);
            telemetry.update();

            /**if (positionSkystone == "left") {
             break;
             } else if (positionSkystone == "center") {
             break;
             } else if (positionSkystone == "right") {
             break;
             }*/
        }
        backLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontLeftWheel.setPower(0);
        frontRightWheel.setPower(0);

    }

    public void DrivewOutRamp(int distance,double straight,double strafe, double target){

        double backleftSpeed,backrightSpeed,frontleftSpeed,frontrightSpeed;


        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //double target = angles.firstAngle;
        double startPosition = backLeftWheel.getCurrentPosition();
        double frontRPower = straight - strafe;
        double frontLPower = straight + strafe;
        double backRPower = straight + strafe;
        double backLPower  = straight - strafe;

        while (((backLeftWheel.getCurrentPosition() < (distance + startPosition)) && (backRightWheel.getCurrentPosition() < (distance + startPosition))) && !isStopRequested()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

            double correction = (target - currentHeading)/100;

            backleftSpeed = backLPower - (correction * 2);
            backrightSpeed = backRPower + (correction * 2);
            frontleftSpeed = frontLPower - (correction * 2);
            frontrightSpeed = frontRPower + (correction * 2);

            backrightSpeed = Range.clip (backrightSpeed,-1,1);
            backleftSpeed = Range.clip (backleftSpeed,-1,1);
            frontleftSpeed = Range.clip (frontleftSpeed,-1,1);
            frontrightSpeed = Range.clip(frontrightSpeed,-1,1);


            backLeftWheel.setPower(backleftSpeed);
            frontLeftWheel.setPower(frontleftSpeed);
            backRightWheel.setPower(backrightSpeed);
            frontRightWheel.setPower(frontrightSpeed);


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("current heading:", angles.firstAngle);
            telemetry.addData("desired heading:", startPosition);
        }

        backLeftWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontRightWheel.setPower(0);


    }

    public void backupwOutRamp(int distance,double straight,double strafe, double target){

        double backleftSpeed,backrightSpeed,frontleftSpeed,frontrightSpeed;


        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //double target = angles.firstAngle;
        double startPosition = backLeftWheel.getCurrentPosition();
        double frontRPower = straight - strafe;
        double frontLPower = straight + strafe;
        double backRPower = straight + strafe;
        double backLPower  = straight - strafe;

        while (((backLeftWheel.getCurrentPosition() > (distance + startPosition)) && (backRightWheel.getCurrentPosition() > (distance + startPosition))) && !isStopRequested()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

            double correction = (target - currentHeading)/100;

            backleftSpeed = backLPower - (correction * 2);
            backrightSpeed = backRPower + (correction * 2);
            frontleftSpeed = frontLPower - (correction * 2);
            frontrightSpeed = frontRPower + (correction * 2);

            backrightSpeed = Range.clip (backrightSpeed,-1,1);
            backleftSpeed = Range.clip (backleftSpeed,-1,1);
            frontleftSpeed = Range.clip (frontleftSpeed,-1,1);
            frontrightSpeed = Range.clip(frontrightSpeed,-1,1);


            backLeftWheel.setVelocity(((backleftSpeed) * 540),DEGREES);
            frontLeftWheel.setVelocity(((frontleftSpeed) * 540),DEGREES);
            backRightWheel.setVelocity(((backrightSpeed) * 540),DEGREES);
            frontRightWheel.setVelocity(((frontrightSpeed) * 540),DEGREES);


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("current heading:", angles.firstAngle);
            telemetry.addData("desired heading:", startPosition);
        }

        backLeftWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontRightWheel.setPower(0);


    }
}