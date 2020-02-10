package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This is NOT an opmode.
 *
 * This class defines all the specific hardware for a three wheel omni-bot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left drive"
 * Motor channel:  Right drive motor:        "right drive"
 * Motor channel:  Rear  drive motor:        "back drive"
 *
 * These motors correspond to three drive locations spaced 120 degrees around a circular robot.
 * Each motor is attached to an omni-wheel. Two wheels are in front, and one is at the rear of the robot.
 *
 * Robot motion is defined in three different axis motions:
 * - Axial    Forward/Backwards      +ve = Forward
 * - Lateral  Side to Side strafing  +ve = Right
 * - Yaw      Rotating               +ve = CCW
 */


public class Robot_OmniDrive
{
    // Private Members
    private LinearOpMode myOpMode;

    public static DcMotor  leftFrontDrive = null;
    public static DcMotor  rightFrontDrive= null;
    public static DcMotor  leftBackDrive  = null;
    public static DcMotor  rightBackDrive = null;

    public static double  driveAxial      = 0 ;   // Positive is forward
    public static double  driveLateral    = 0 ;   // Positive is right
    public static double  driveYaw        = 0 ;   // Positive is CCW

    /* Constructor */
    public Robot_OmniDrive(){

    }


    /* Initialize standard Hardware interfaces */
    public void initDrive(LinearOpMode opMode) {

        // Save reference to Hardware map
        myOpMode = opMode;

        // Define and Initialize Motors
        leftFrontDrive        = myOpMode.hardwareMap.get(DcMotor.class, "Front_left_wheel");
        rightFrontDrive       = myOpMode.hardwareMap.get(DcMotor.class, "Front_right_wheel");
        leftBackDrive         = myOpMode.hardwareMap.get(DcMotor.class, "Back_left_wheel");
        rightBackDrive        = myOpMode.hardwareMap.get(DcMotor.class,"Back_right_wheel");

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE); // Positive input rotates counter clockwise
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);// Positive input rotates counter clockwise
        //backDrive.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise

        //use RUN_USING_ENCODERS because encoders are installed.
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stop all robot motion by setting each axis value to zero
        moveRobot(0,0,0) ;
    }

    public void manualDrive()  {
        // In this mode the Left stick moves the robot fwd & back, and Right & Left.
        // The Right stick rotates CCW and CW.

        //  (note: The joystick goes negative when pushed forwards, so negate it)
        if ((myOpMode.gamepad1.left_stick_y < -.1 ) || (myOpMode.gamepad1.left_stick_x < -.1 ) || (myOpMode.gamepad1.right_stick_x < -.1 ) || (myOpMode.gamepad1.left_stick_y > .1 ) || (myOpMode.gamepad1.left_stick_x < .1 ) || (myOpMode.gamepad1.right_stick_x > .1))
        setAxial(-myOpMode.gamepad1.left_stick_y);
        setLateral(myOpMode.gamepad1.left_stick_x);
        setYaw(-myOpMode.gamepad1.right_stick_x);
    }


    /***
     * void moveRobot(double axial, double lateral, double yaw)
     * Set speed levels to motors based on axes requests
     * @param axial     Speed in Fwd Direction
     * @param lateral   Speed in lateral direction (+ve to right)
     * @param yaw       Speed of Yaw rotation.  (+ve is CCW)
     */
    public void moveRobot(double axial, double lateral, double yaw) {
        setAxial(axial);
        setLateral(lateral);
        setYaw(yaw);
        moveRobot();
    }

    /***
     * void moveRobot()
     * This method will calculate the motor speeds required to move the robot according to the
     * speeds that are stored in the three Axis variables: driveAxial, driveLateral, driveYaw.
     * This code is setup for a three wheeled OMNI-drive but it could be modified for any sort of omni drive.
     *
     * The code assumes the following conventions.
     * 1) Positive speed on the Axial axis means move FORWARD.
     * 2) Positive speed on the Lateral axis means move RIGHT.
     * 3) Positive speed on the Yaw axis means rotate COUNTER CLOCKWISE.
     *
     * This convention should NOT be changed.  Any new drive system should be configured to react accordingly.
     */
    public void moveRobot() {
        // calculate required motor speeds to acheive axis motions
        // NOTE: both left motors are reversed
        double rbPower = -driveAxial - driveLateral - driveYaw;
        double rfPower = -driveAxial + driveLateral - driveYaw;
        double lbPower = -driveAxial + driveLateral + driveYaw;
        double lfPower = -driveAxial - driveLateral + driveYaw;
        // normalize all motor speeds so no values exceeds 100%.
        double max = Math.max(Math.abs(rbPower), Math.abs(rfPower));
        max = Math.max(max, Math.abs(lbPower));
        max = Math.max(max,Math.abs(lfPower));
        if (max > 1.0)
        {
            rbPower /= max;
            rfPower /= max;
            lbPower /= max;
            lfPower /= max;
        }

        // Set drive motor power levels.
        rightBackDrive.setPower(rbPower);
        rightFrontDrive.setPower(rfPower);
        leftFrontDrive.setPower(lfPower);
        leftBackDrive.setPower(lbPower);

        // Display Telemetry
        //myOpMode.telemetry.addData("Axes  ", "A[%+5.2f], L[%+5.2f], Y[%+5.2f]", driveAxial, driveLateral, driveYaw);
        //myOpMode.telemetry.addData("Wheels", "RB[%+5.2f], RF[%+5.2f], LB[%+5.2f], LF[%+5.2f]", rbPower, rfPower, lbPower,lfPower);
    }


    public void setAxial(double axial)      {driveAxial = Range.clip(axial, -1, 1);}
    public void setLateral(double lateral)  {driveLateral = Range.clip(lateral, -1, 1); }
    public void setYaw(double yaw)          {driveYaw = Range.clip(yaw, -1, 1); }


    /***
     * void setMode(DcMotor.RunMode mode ) Set all drive motors to same mode.
     * @param mode    Desired Motor mode.
     */
    public void setMode(DcMotor.RunMode mode ) {
        rightBackDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
    }
}

