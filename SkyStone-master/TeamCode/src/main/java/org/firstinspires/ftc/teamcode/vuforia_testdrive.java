package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous (name = "vuforia testdrive")
//@Disabled
public class vuforia_testdrive extends LinearOpMode {

    private DcMotor backLeftWheel, backRightWheel, frontLeftWheel, frontRightWheel, linearLift, linearLift2;
    private Servo CLAW, trayServoL, trayServoR;
    BNO055IMU imu;
    Orientation angles;
    DistanceSensor Distance;
    DigitalChannel Touch;
    private ElapsedTime runtime = new ElapsedTime();

    /**
     * This code (below) sets the parameters for vuforia blocks
     */
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
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

    /**
     * This code ^ sets the parameters for vuforia blocks
     */
    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        backLeftWheel = hardwareMap.get(DcMotor.class, "Back_left_wheel");
        backRightWheel = hardwareMap.get(DcMotor.class, "Back_right_wheel");
        frontLeftWheel = hardwareMap.get(DcMotor.class, "Front_left_wheel");
        frontRightWheel = hardwareMap.get(DcMotor.class, "Front_right_wheel");
        CLAW = hardwareMap.servo.get("CLAW");
        linearLift = hardwareMap.get(DcMotor.class, "linearLift");
        linearLift2 = hardwareMap.get(DcMotor.class, "linearLift2");
        trayServoL = hardwareMap.get(Servo.class, "trayServoL");
        trayServoR = hardwareMap.get(Servo.class, "trayServoR");
        Distance = hardwareMap.get(DistanceSensor.class, "Distance");
        Touch = hardwareMap.get(DigitalChannel.class, "Touch");
        imu.initialize(parameters);

        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Touch.setMode(DigitalChannel.Mode.INPUT);

        trayServoR.setPosition(1);
        trayServoL.setPosition(0);

        CLAW.setPosition(0);

        while ((!imu.isGyroCalibrated()) && !isStopRequested()) {

        }

        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //This code ^ virtually tells the vuforia program where everything is


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        vuforiaparameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforiaparameters.cameraDirection = CAMERA_CHOICE;

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
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 5.93f * mmPerInch;   // eg: Camera is 5.93 Inches above ground when portrait, and 7.6875 when landscape
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

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


        telemetry.addData("Status:", "initialized");
        telemetry.update();

        //waitForStart();



        //CameraDevice.getInstance().setFlashTorchMode(true);

        waitForStart();

        targetsSkyStone.activate();

        Drive(400,.7,0,0);

        Drive(70,0,.4,0);

        while (!isStopRequested() && (((positionSkystone != "left") && positionSkystone != "center") && positionSkystone != "right")) {

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
                if (yPosition >= -6) {
                    positionSkystone = "right";
                } else if (yPosition <= -10) {
                    positionSkystone = "center";
                }

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }else {
                //positionSkystone = "left";
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

        telemetry.addData("Skystone position:", positionSkystone);
        telemetry.update();
        sleep(5000);
        targetsSkyStone.deactivate();

        //CameraDevice.getInstance().setFlashTorchMode(false);

        if (positionSkystone == "left") {

            Lift(1120,1);

            Drive(560,0,-.7,0);

            Drop(140,.5);

            CLAW.setPosition(0);
            sleep(500);

            backup(-140,-.4,0,0);

            backup(-200,0,-.7,0);
            backup(-1210,0,-1,0);
            backup(-200,0,-.7,0);
        } else if (positionSkystone == "center") {

            Lift(1120,1);

            Drive(460,0,-.7,0);

            Drop(140,.5);

            CLAW.setPosition(0);
            sleep(500);

            backup(-140,-.4,0,0);

            backup(-200,0,-.7,0);
            backup(-1310,0,-1,0);
            backup(-200,0,-.7,0);
        } else if (positionSkystone == "right") {

            Lift(1120,1);

            Drive(360,0,-.7,0);

            Drop(140,.5);

            CLAW.setPosition(0);
            sleep(500);

            backup(-140,-.4,0,0);

            backup(-200,0,-.7,0);
            backup(-1410,0,-1,0);
            backup(-200,0,-.7,0);
        }

        Lift(840, 1);

        DrivewDistance(8, .5, 0, 90);
        Drive(32, .4, 0, 90);

        trayServoL.setPosition(1);
        trayServoR.setPosition(0);
        sleep(500);

        DrivewTouch(-1, 0, 90, 3);
        Drop(0, .5);
        CLAW.setPosition(1);
        Lift(1320, 1);

        trayServoL.setPosition(0);
        trayServoR.setPosition(1);

        Drive(32, .5, 0, 90);
        Drive(500, 0, 1, 90);
        Drop(0, .5);
        Drive(140, .7, 0, 90);
        Drive(280, 0, -.7, 90);
        Drive(500, 0, 1, 90);

        CLAW.setPosition(0);

        sleep(1000);

        runtime.reset();
    }

    public void Drive(int distance, double straight, double strafe, int target) throws InterruptedException {

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

        while (((backLeftWheel.getCurrentPosition() < (distance + startPosition)) && (backRightWheel.getCurrentPosition() < (distance + startPosition))) && !isStopRequested()) {
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

    public void Lift(int Height, double power) throws InterruptedException {
        while ((linearLift.getCurrentPosition() >= -Height) && !isStopRequested()) {
            linearLift.setPower(-power);
            linearLift2.setPower(-power);
        }
        linearLift.setPower(0);
        linearLift2.setPower(0);
    }

    public void Drop(int Height, double power) throws InterruptedException {
        while ((linearLift.getCurrentPosition() <= -Height) && !isStopRequested()) {
            linearLift.setPower(power);
            linearLift2.setPower(power);
        }
        linearLift.setPower(0);
        linearLift2.setPower(0);
    }

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

        while (((backLeftWheel.getCurrentPosition() > (distance + startPosition)) && (backRightWheel.getCurrentPosition() > (distance + startPosition))) && !isStopRequested()) {
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

    public void DrivewDistance(double range, double straight, double strafe, int target) throws InterruptedException {

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

        while (((Touch.getState() == true) && !isStopRequested()) && runtime.time() < timeLimit) {
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

    public void trackskystone() throws InterruptedException {

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
                }*/
        }
    }
}