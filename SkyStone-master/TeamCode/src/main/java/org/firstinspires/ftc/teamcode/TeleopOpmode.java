/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Robot_OmniDrive;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.Robot_Navigation.AXIAL_GAIN;
import static org.firstinspires.ftc.teamcode.Robot_Navigation.LATERAL_GAIN;
import static org.firstinspires.ftc.teamcode.Robot_Navigation.YAW_GAIN;
import static org.firstinspires.ftc.teamcode.Robot_Navigation.myOpMode;
import static org.firstinspires.ftc.teamcode.Robot_Navigation.relativeBearing;
import static org.firstinspires.ftc.teamcode.Robot_Navigation.robotBearing;
import static org.firstinspires.ftc.teamcode.Robot_Navigation.robotX;
import static org.firstinspires.ftc.teamcode.Robot_Navigation.robotY;
import static org.firstinspires.ftc.teamcode.Robot_Navigation.targetBearing;
import static org.firstinspires.ftc.teamcode.Robot_Navigation.targetFound;
import static org.firstinspires.ftc.teamcode.Robot_Navigation.targetName;
import static org.firstinspires.ftc.teamcode.Robot_Navigation.targetRange;
import static org.firstinspires.ftc.teamcode.Robot_OmniDrive.driveAxial;
import static org.firstinspires.ftc.teamcode.Robot_OmniDrive.driveLateral;
import static org.firstinspires.ftc.teamcode.Robot_OmniDrive.driveYaw;
import static org.firstinspires.ftc.teamcode.Robot_OmniDrive.leftBackDrive;
import static org.firstinspires.ftc.teamcode.Robot_OmniDrive.leftFrontDrive;
import static org.firstinspires.ftc.teamcode.Robot_OmniDrive.rightBackDrive;
import static org.firstinspires.ftc.teamcode.Robot_OmniDrive.rightFrontDrive;

/**
 * This example is designed to show how to identify a target, get the robot's position, and then plan
 * and execute an approach path to the target.
 *
 * This OpMode uses two "utility" classes to abstract (hide) the hardware and navigation GUTs.
 * These are:  Robot_OmniDrive and Robot_Navigation.
 *
 * This LinearOpMode uses basic hardware and nav calls to drive the robot in either manual or auto mode.
 * AutoMode is engaged by pressing and holding the Left Bumper.  Release the Bumper to return to Manual Mode.
 *
 *  *ManualMode* simply uses the joysticks to move the robot in three degrees of freedom.
 *  - Left stick X (translate left and right)
 *  - Left Stick Y (translate forward and backwards)
 *  - Right Stick X (rotate CW and CCW)
 *
 *  *AutoMode* will approach the image target and attempt to reach a position directly in front
 *  of the center line of the image, with a predefined stand-off distance.
 *
 *  To simplify this example, a gyro is NOT used.  Therefore there is no attempt being made to stabilize
 *  strafing motions, or to perform field-centric driving.
 *
 */

@TeleOp(name="Vuforia Tracking Demo", group="main")
public class TeleopOpmode extends LinearOpMode {

    final double TARGET_DISTANCE = 400.0;    // Hold robot's center 400 mm from target

    /* Declare OpMode members. */
    Robot_OmniDrive robot = new Robot_OmniDrive();   // Use Omni-Directional drive system
    Robot_Navigation nav = new Robot_Navigation();  // Use Image Tracking library
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.FRONT;
    private VuforiaLocalizer vuforia = null;
    private VuforiaBase targetSkystone;
    VuforiaTrackables targetsSkyStone;
    private boolean targetVisible = false;
    private OpenGLMatrix lastLocation = null;
    private static final float mmPerInch = 25.4f;
    //VectorF trans = location.getTranslation();
    //Orientation rot = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    boolean closeEnough;






    @Override
    public void runOpMode() {

        // Initialize the robot and navigation
        robot.initDrive(this);
        telemetry.addLine("initialized mecanum drive");
        telemetry.update();
        //sleep(1000);

        // Save reference to OpMode and Hardware map
        //myOpMode = opMode;
        //myRobot = robot;

        /**
         * Start up Vuforia, telling it the id of the view that we wish to use as the parent for
         * the camera monitor.
         * We also indicate which camera on the RC that we wish to use.
         */

        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);  // Use this line to see camera display
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);                             // OR... Use this line to improve performance

        // Get your own Vuforia key at  https://developer.vuforia.com/license-manager
        // and paste it here...
        parameters.vuforiaLicenseKey = "AYVmAfT/////AAABmXwAtv5eVkf0ggIaxYGXWVlcP0vBPM4m6TySqTzdDarE/KT2b9MWrfLJyQ4XzI0BcA75i53Ym2EQglAqnTfwN12JaI+9Qj4Wk4A4zTdz3f2dtZCMaNuy+hwDGVag9mDxviLt898OvVBj48Mrjm7ogIlkUtrkqACsvRt53PT0ee2QAW9bHkRAu4FS8XbBBq27o4GhsTIO7JfbRfx8M8KEzMHUBgnIoYxooUXv164KZ8TmkJRaj5r1IUB7woRh7bMU4YO8SP6g4M6ntjMCUPxmxZivm28H0u7yPExy0syFNIIsbqyMFYJaISXo0U3WM87obdR2nOvQvTdGjPX8J0bLyBm/ii6M/GmGI2gmA7hL+qFe";

        parameters.cameraDirection = CAMERA_CHOICE;
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        /**
         * Load the data sets that for the trackable objects we wish to track.
         * These particular data sets are stored in the 'assets' part of our application
         * They represent the four image targets used in the 2016-17 FTC game.
         */
        telemetry.addLine("pre-target init");
        telemetry.update();
        //sleep(1000);

        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        /**VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
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

         telemetry.addLine("after target init");
         telemetry.update();
         //sleep(5100);

         /** For convenience, gather together all the trackable objects in one easily-iterable collection */

        //VuforiaTrackable target = targetsSkyStone.get(0);

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        // create an image translation/rotation matrix to be used for all images
        // Essentially put all the image centers 6" above the 0:0:0 origin,
        // but rotate them so they along the -X axis.
        //OpenGLMatrix targetOrientation = OpenGLMatrix

        telemetry.addLine("pre-stone placement");
        telemetry.update();
        //sleep(1000);
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, 50)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, -90)));

        /**OpenGLMatrix targetOrientation = OpenGLMatrix.translation(0,0,50)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC,AxesOrder.XYZ,
                DEGREES,90,0,-90));*/

        telemetry.addLine("after stone placement");
        telemetry.update();
        //sleep(1000);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  If we consider that the camera and screen will be
         * in "Landscape Mode" the upper portion of the screen is closest to the front of the robot.
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

        telemetry.addLine("pre-camera init");
        telemetry.update();
        //sleep(1000);

        final int CAMERA_FORWARD_DISPLACEMENT = 180;   // Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 156;   // Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = -75;     // Camera is ON the robots center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZX,
                        AngleUnit.DEGREES, CAMERA_CHOICE == VuforiaLocalizer.CameraDirection.FRONT ? 90 : -90, 0, 0));

        telemetry.addLine("after camera init");
        telemetry.update();
        //sleep(1000);
        // Set the all the targets to have the same location and camera orientation

        telemetry.addLine("initializing this this thingy");
        telemetry.update();
        //sleep(1000);
        for (VuforiaTrackable trackable : allTrackables) {
            //trackable.setLocation(targetOrientation);
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        // Activate Vuforia (this takes a few seconds)
        telemetry.addLine("activating vuforia");
        telemetry.update();
        //sleep(1000);
        nav.activateTracking();

        //location = listener.getUpdatedRobotLocation();
        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("pre-!isStarted() loop");
        telemetry.update();
        //sleep(1000);
        while (!isStarted()) {
            // Prompt User
            targetsSkyStone.activate();
            telemetry.addData(">", "Press start");

            // Display any Nav Targets while we wait for the match to start
            //nav.targetsAreVisible();
            //nav.addNavTelemetry();

            CameraDevice.getInstance().setFlashTorchMode(true);

            // check all the trackable targets to see which one (if any) is visible.
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
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //targetsSkyStone.activate();
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    //telemetry.addData("Visible Target", trackable.getName());
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
            /**if (targetVisible) {
             // express position (translation) of robot in inches.
             VectorF translation = lastLocation.getTranslation();
             telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
             translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

             // express the rotation of the robot in degrees.
             Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
             telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
             }
             else {
             telemetry.addData("Visible Target", "none");
             }*/
            telemetry.update();
            // this code ^ tracks whether the stone is being seen or not

            //telemetry.addData(">", "Press Left Bumper to track target");

            //sleep(2000);

            // auto drive or manual drive?
            // In auto drive, the robot will approach any target it can see and then press against it
            // In manual drive the robot responds to the Joystick.
            if (targetVisible && gamepad1.left_bumper) {
                /**telemetry.addLine("pre-calculation");
                telemetry.update();
                //sleep(5000);*/

                VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) stoneTarget.getListener();
                OpenGLMatrix location;

                if ((stoneTarget != null) && (listener != null) && listener.isVisible() && !closeEnough) {
                    targetFound = true;
                    targetName = stoneTarget.getName();

                    // If we have an updated robot location, update all the relevant tracking information
                    location  = listener.getUpdatedRobotLocation();
                    if (location != null) {

                        // Create a translation and rotation vector for the robot.
                        VectorF trans = location.getTranslation();
                        Orientation rot = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                        // Robot position is defined by the standard Matrix translation (x and y)
                        robotX = trans.get(0);
                        robotY = trans.get(1);

                        // Robot bearing (in +vc CCW cartesian system) is defined by the standard Matrix z rotation
                        robotBearing = rot.thirdAngle;

                        // target range is based on distance from robot position to origin.
                        targetRange = Math.hypot(robotX, robotY);

                        // target bearing is based on angle formed between the X axis to the target range line
                        targetBearing = Math.toDegrees(-Math.asin(robotY / targetRange));

                        // Target relative bearing is the target Heading relative to the direction the robot is pointing.
                        relativeBearing = targetBearing - robotBearing;
                    }
                    targetFound = true;
                }
                else  {
                    // Indicate that there is no target visible
                    targetFound = false;
                    targetName = "None";
                }

                //return targetFound;


                targetFound = true;
                /**telemetry.addLine("pre-drive");
                telemetry.update();
                //sleep(5000);*/



                // Priority #1 Rotate to always be pointing at the target (for best target retention).
                //double Y  = (relativeBearing * YAW_GAIN);

                // Priority #2  Drive laterally based on distance from X axis (same as y value)
                //double L  = (-(robotY * LATERAL_GAIN) * 2);

                // Priority #3 Drive forward based on the desiredHeading target standoff distance
                double A  = (-(robotX + 400) * AXIAL_GAIN);

                // Send the desired axis motions to the robot hardware.
                robot.driveAxial = Range.clip(A,-1,1);
                //robot.driveLateral = Range.clip(L,-1,1);
                //robot.driveYaw = Range.clip(Y,-1,1);

                // Determine if we are close enough to the target for action.

                if ((driveAxial < .2) && (driveAxial > 0)){
                   driveAxial = .2;
                } else if ((driveAxial > -.2) && (driveAxial < 0)){
                    driveAxial = -.2;
                } else {
                    driveAxial = Range.clip(A,-1,1);
                }

                closeEnough = ( (Math.abs(robotX + 400) < 20)/** &&
                        (Math.abs(robotY) < 5)*/);

            } else {
                // Drive the robot using the joysticks
                robot.manualDrive();
            }

            // Build telemetry messages with Navigation Information;
            //nav.addNavTelemetry();

            if (targetVisible) {
                // Display the current visible target name, robot info, target info, and required robot action.
                /**myOpMode.*/telemetry.addData("Visible", targetName);
                /**myOpMode.*/telemetry.addData("Robot", "[X]:[Y] (B) [%5.0fmm]:[%5.0fmm] (%4.0f°)",
                        robotX, robotY, robotBearing);
                /**myOpMode.*/
                telemetry.addData("Target", "[R] (B):(RB) [%5.0fmm] (%4.0f°):(%4.0f°)",
                        targetRange, targetBearing, relativeBearing);
                /**myOpMode.*/
                telemetry.addData("- Turn    ", "%s %4.0f°", relativeBearing < 0 ? ">>> CW " : "<<< CCW", Math.abs(relativeBearing));
                /**myOpMode.*/
                telemetry.addData("- Strafe  ", "%s %5.0fmm", robotY < 0 ? "LEFT" : "RIGHT", Math.abs(robotY));
                /**myOpMode.*/telemetry.addData("- Distance", "%5.0fmm", Math.abs(robotX));
                telemetry.update();

            } else {
                /**myOpMode.*/telemetry.addData("Visible", "- - - -");
                telemetry.update();

            }

            //  Move the robot according to the pre-determined axis motions
            //driveAxial = .5;
            //driveYaw = .5;
            robot.moveRobot();
            telemetry.update();


        }
        targetsSkyStone.deactivate();
        telemetry.addData(">", "Shutting Down. Bye!");
        telemetry.update();
    }
}
    /**public void calculateVuforia (int targetId) {

        VuforiaTrackable target = targetsSkystone.get(0);
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        OpenGLMatrix location  = null;

        // if we have a target, look for an updated robot position
        if ((target != null) && (listener != null) && listener.isVisible()) {
            targetFound = true;
            targetName = target.getName();

            // If we have an updated robot location, update all the relevant tracking information
            location  = listener.getUpdatedRobotLocation();
             if (location != null) {

                // Create a translation and rotation vector for the robot.
                VectorF trans = location.getTranslation();
                Orientation rot = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Robot position is defined by the standard Matrix translation (x and y)
                robotX = trans.get(0);
                robotY = trans.get(1);

                // Robot bearing (in +vc CCW cartesian system) is defined by the standard Matrix z rotation
                robotBearing = rot.thirdAngle;

                // target range is based on distance from robot position to origin.
                targetRange = Math.hypot(robotX, robotY);

                // target bearing is based on angle formed between the X axis to the target range line
                targetBearing = Math.toDegrees(-Math.asin(robotY / targetRange));

                // Target relative bearing is the target Heading relative to the direction the robot is pointing.
                relativeBearing = targetBearing - robotBearing;
            }
            targetFound = true;
        }
        else  {
            // Indicate that there is no target visible
            targetFound = false;
            targetName = "None";
        }

        //return targetFound;
    }*/


