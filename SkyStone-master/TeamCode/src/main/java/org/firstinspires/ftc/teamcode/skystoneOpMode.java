package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.Gyroscope;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/**

 * Created by Boen on 10-10-19

*/



@TeleOp
@Disabled





public class skystoneOpMode extends LinearOpMode {

    private Gyroscope imu;

    private DcMotorEx backLeftWheel, backRightWheel, frontLeftWheel, frontRightWheel;

    private DcMotor linearLift;

    private Servo CLAW;

    boolean upButton;
    boolean downButton;
    double speed;
    double rotation;
    double strafe;
    int goalLiftHeight = 0;
    int desiredPosition;

    @Override

    public void runOpMode() throws InterruptedException {


        imu = hardwareMap.get(Gyroscope.class, "imu");
        backLeftWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Back_left_wheel");
        backRightWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Back_right_wheel");
        frontLeftWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Front_left_wheel");
        frontRightWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Front_right_wheel");
        linearLift = (DcMotorEx) hardwareMap.get(DcMotor.class, "linearLift");
        CLAW = hardwareMap.servo.get("CLAW");

        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        backRightWheel.setDirection(DcMotor.Direction.REVERSE);

        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);

        linearLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLift.setTargetPosition(0);
        linearLift.setPower(.5);
        linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addData("Status", "Initialized");


        telemetry.update();


        //Wait for the game to start (driver presses PLAY)


        waitForStart();

        //run until the end of the match (driver presses STOP)


        while (opModeIsActive()) {

            // Drive code

            speed = this.gamepad1.left_stick_y;

            rotation = this.gamepad1.left_stick_x;

            strafe = -this.gamepad1.right_stick_x;


            backLeftWheel.setPower(-speed + strafe - rotation);

            backRightWheel.setPower(speed + strafe - rotation);

            frontLeftWheel.setPower(-speed + strafe + rotation);

            frontRightWheel.setPower(speed + strafe + rotation);


            //CLAAAAW code

            if (this.gamepad2.x) {

                CLAW.setPosition(0);

            } else if (this.gamepad2.b) {

                CLAW.setPosition(1);

            }


            if (this.gamepad2.y) {
                if (!upButton) {

                    goalLiftHeight += 1;
                    upButton = true;

                } else {
                }

            } else {
                upButton = false;
            }


            if (this.gamepad2.a) {
                if (!downButton) {
                    goalLiftHeight -= 1;
                    downButton = true;
                } else {
                }
            } else {
                downButton = false;
            }

            if (goalLiftHeight > 3) {
                goalLiftHeight = 3;
            }
            if (goalLiftHeight < 0) {
                goalLiftHeight = 0;
            }

            //going up
            if (this.gamepad2.y) {

                if (goalLiftHeight == 1) {

                    desiredPosition = 1120;

                   // while (linearLift.isBusy()) {
                    //}
                    //Do nothing, this helps to make the motor keep on moving to its target position without any
                    //interruptions.
                } else if (goalLiftHeight == 2) {

                   desiredPosition = 2240;
                    //linearLift.setPower(.25);

                    //linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    //while (linearLift.isBusy()) {
                    //}
                    //Do nothing, this helps to make the motor keep on moving to its target position without any
                    //interruptions

                } else if (goalLiftHeight == 3) {

                    desiredPosition = 3360;
                    //linearLift.setPower(.25);
                    //linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    //while (linearLift.isBusy()) {
                    //}
                    //Do nothing, this helps to make the motor keep on moving to its target position without any
                    //interruptions.
                }

            }

            if (this.gamepad2.a) {

                if (goalLiftHeight == 0) {

                    // How many degrees per second should it go up
                    // linearLift.setVelocity(60, AngleUnit.DEGREES);

                    desiredPosition = 0;
                    //linearLift.setPower(.25);
                    //linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //while (linearLift.isBusy()) {
                    //}
                    //Do nothing, this helps to make the motor keep on moving to its target position without any
                    //interruptions.
                } else if (goalLiftHeight == 1) {

                   desiredPosition = 1120;
                    //linearLift.setPower(.25);
                    //linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    //while (linearLift.isBusy()) {
                    //}
                    //Do nothing, this helps to make the motor keep on moving to its target position without any
                    //interruptions.
                } else if (goalLiftHeight == 2) {

                    desiredPosition = 2240;
                    //linearLift.setPower(.25);
                    //linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    //while (linearLift.isBusy()) {
                    //}
                    //Do nothing, this helps to make the motor keep on moving to its target position without any
                    //interruptions
                }

            } else {

                // Hold Current Position

                //linearLift.setVelocity(0, AngleUnit.DEGREES);

                linearLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            }
            //linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
    }
}