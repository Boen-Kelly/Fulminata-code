package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.Gyroscope;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;




/**
 * Created by Boen on 10-10-19
 */


@TeleOp(name = "Skystone op mode", group = "competition modes")


public class upDownMode extends LinearOpMode {

    private Gyroscope imu;

    private DcMotorEx backLeftWheel, backRightWheel, frontLeftWheel, frontRightWheel, linearLift, linearLift2;

    private Servo CLAW, trayServoL, trayServoR,Capstone;
    double startPosition;
    int goalLiftHeight = 0;
    double liftMultiplier = 1;
    boolean open;
    double totalTicks;
    private GamepadEx driverGamepad, operatorGamepad;
    private ButtonReader upButton, downButton;
    int mockGoalLiftHeight;
    double speed;
    double rotation;
    double strafe;
    double multiplier = 0;
    boolean buttonPressed = false;
    boolean regMode;
    int goalLiftHeight2;

    @Override


    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(Gyroscope.class, "imu");
        backLeftWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Back_left_wheel");
        backRightWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Back_right_wheel");
        frontLeftWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Front_left_wheel");
        frontRightWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Front_right_wheel");
        linearLift = (DcMotorEx) hardwareMap.get(DcMotor.class, "linearLift");
        linearLift2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "linearLift2");
        CLAW = hardwareMap.servo.get("CLAW");
        trayServoL = hardwareMap.servo.get("trayServoL");
        trayServoR = hardwareMap.servo.get("trayServoR");
        Capstone = hardwareMap.get(Servo.class, "Capstone");
        //xbutton = new ButtonReader(operatorGamepad, GamepadKeys.Button.X);
        operatorGamepad = new GamepadEx(gamepad2);
        upButton = new ButtonReader(operatorGamepad, GamepadKeys.Button.DPAD_UP);
        downButton = new ButtonReader(operatorGamepad, GamepadKeys.Button.DPAD_DOWN);


        linearLift.setTargetPositionTolerance(40);
        linearLift2.setTargetPositionTolerance(40);

        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        backRightWheel.setDirection(DcMotor.Direction.REVERSE);

        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);

        linearLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        linearLift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        linearLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linearLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        CLAW.setPosition(0);
        trayServoL.setPosition(0);
        trayServoR.setPosition(1);

        telemetry.addData("Status", "Initialized");

        telemetry.update();


        //Wait for the game to start (driver presses PLAY)


        waitForStart();


        //run until the end of the match (driver presses STOP)




        while (opModeIsActive()) {
            upButton.readValue();
            downButton.readValue();

            // Drive code

            speed = this.gamepad1.left_stick_y * multiplier;

            rotation = -this.gamepad1.left_stick_x * multiplier;

            strafe = -this.gamepad1.right_stick_x * multiplier;


            backLeftWheel.setPower(speed + strafe - rotation);

            backRightWheel.setPower(-speed + strafe - rotation);

            frontLeftWheel.setPower(speed + strafe + rotation);

            frontRightWheel.setPower(-speed + strafe + rotation);

            if (gamepad1.left_trigger > .5) {
                multiplier = .4;
            } else {
                multiplier = 1;
            }


            // CLAAAAW code
            if (this.gamepad2.a) {

                CLAW.setPosition(0);

            } else if (this.gamepad2.y) {

                CLAW.setPosition(1);

            }

            if(this.gamepad1.left_bumper){
                trayServoL.setPosition(1);
                trayServoR.setPosition(0);
            }else{
                trayServoL.setPosition(0);
                trayServoR.setPosition(1);
            }

            //**
            //This code is for slowing down the lift, if we don't use it, then just un-comment out this code ^^ above

            if (gamepad2.right_trigger > .25) {
                liftMultiplier = .5;
            } else {
                liftMultiplier = 1;
            }
            //*/



            int currentPosition = linearLift.getCurrentPosition();
            double stickHeight = this.gamepad2.left_stick_y;

            if (currentPosition <= -22400) {
                if (stickHeight > .25) {
                    linearLift.setPower(stickHeight * liftMultiplier);
                    linearLift2.setPower(stickHeight * liftMultiplier);
                } else {
                    linearLift.setPower(0);
                    linearLift2.setPower(0);
                }
            } else if (currentPosition >= 0) {
                if (stickHeight < -.25) {
                    linearLift.setPower(stickHeight * liftMultiplier);
                    linearLift2.setPower(stickHeight * liftMultiplier);
                } else {
                    linearLift.setPower(0);
                    linearLift2.setPower(0);
                }
            } else {
                if ((stickHeight > .25) && !(currentPosition >= 0)) {
                    linearLift.setPower(stickHeight * liftMultiplier);
                    linearLift2.setPower(stickHeight * liftMultiplier);
                } else if ((stickHeight < -0.25) && !(currentPosition <= -22400)) {
                    linearLift.setPower(stickHeight * liftMultiplier);
                    linearLift2.setPower(stickHeight * liftMultiplier);
                }
            }


            if (((gamepad2.left_trigger > .5) && gamepad2.x)){
                if (open){
                    Capstone.setPosition(1);
                } else if (open == false){
                    Capstone.setPosition(0);
                }

            }



            if (upButton.wasJustPressed()) {
                mockGoalLiftHeight++;
            } else if (downButton.wasJustPressed()) {
                mockGoalLiftHeight -= 1;
            }

            if (mockGoalLiftHeight > 7) {
                mockGoalLiftHeight = 7;
            } else if (mockGoalLiftHeight < 0) {
                mockGoalLiftHeight = 0;
            }

            // Alternative to below if statements

            /*switch (mockGoalLiftHeight) {
                case 0:
                    goalLiftHeight = 0;
                    break;
                case 1:
                    goalLiftHeight = -560;
                    break;
                case 2:
                    goalLiftHeight = -1120;
                    break;
                case 3:
                    goalLiftHeight = -1680;
                    break;
                case 4:
                    goalLiftHeight = -2240;
                    break;

                default:
                    goalLiftHeight = 0;
                    break;
            }*/

            if (upButton.wasJustPressed() || downButton.wasJustPressed()){
                buttonPressed = true;
            }

            if ((mockGoalLiftHeight == 0) && buttonPressed) {
                goalLiftHeight2 = 0;
                goalLiftHeight = 0;
            } else if ((mockGoalLiftHeight == 1) && buttonPressed) {
                goalLiftHeight2 = -5600;
                goalLiftHeight = -5600;
            } else if ((mockGoalLiftHeight == 2) && buttonPressed) {
                goalLiftHeight2 = -11200;
                goalLiftHeight = -11200;
            } else if ((mockGoalLiftHeight == 3) && buttonPressed) {
                goalLiftHeight2 = -16800;
                goalLiftHeight = -16800;
            } else if ((mockGoalLiftHeight == 4) && buttonPressed) {
                goalLiftHeight2 = -22400;
                goalLiftHeight = -22400;
            }else if ((mockGoalLiftHeight == 5) && buttonPressed) {
                goalLiftHeight2 = -22400;
                goalLiftHeight = -22400;
            }else if ((mockGoalLiftHeight == 6) && buttonPressed) {
                goalLiftHeight = -22400;
                goalLiftHeight2 = -22400;
            }else if ((mockGoalLiftHeight == 7) && buttonPressed) {
                goalLiftHeight = -22400;
                goalLiftHeight2 = -22400;
            } else {
                goalLiftHeight = linearLift.getCurrentPosition();
                goalLiftHeight2 = linearLift2.getCurrentPosition();
            }





            if (gamepad2.left_stick_button) {
                linearLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linearLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else if ((gamepad2.left_stick_y > .25) || (gamepad2.left_stick_y < -.25)){
                linearLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                linearLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                buttonPressed = false;

            } else {
                linearLift.setTargetPosition(goalLiftHeight);
                linearLift2.setTargetPosition(goalLiftHeight2);
                linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearLift.setPower(1);
                linearLift2.setPower(1);
            }

            telemetry.addData("CLAAAAAW position", CLAW.getPosition());
            telemetry.addData("Target Power", speed);
            telemetry.addData("Motor Power", backLeftWheel.getPower());
            telemetry.addData("Status", "Running");
            telemetry.addData("Linear lift height", linearLift.getCurrentPosition());
            telemetry.addData("linear lift 2 height:", linearLift2.getCurrentPosition());
            telemetry.addData("linear lift 2 target height:", linearLift2.getTargetPosition());
            telemetry.addData("target height:", goalLiftHeight);
            telemetry.addData("mockGoalLiftHeight:", mockGoalLiftHeight);
            telemetry.addData("linear lift mode:",linearLift.getMode());
            telemetry.addData("stick height:",gamepad2.left_stick_y);

            telemetry.update();


        }


    }

}
