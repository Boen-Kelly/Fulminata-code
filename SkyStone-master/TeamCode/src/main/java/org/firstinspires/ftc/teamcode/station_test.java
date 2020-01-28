package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

@TeleOp
@Disabled
public class station_test extends LinearOpMode {

    private DcMotor backLeftWheel, backRightWheel, frontLeftWheel, frontRightWheel;
    DcMotorEx  linearLift, linearLift2;
    private Servo CLAW, trayServoL, trayServoR;
    BNO055IMU imu;
    Orientation angles;
    DistanceSensor Distance;
    DigitalChannel Touch;
    private ElapsedTime runtime = new ElapsedTime();
    double startPosition;
    int mockGoalLiftheight = 0;
    boolean upButton;
    boolean downButton;
    double liftMultiplier = 1;
    int goalLiftHeight = 0;
    int prevGoalLiftHeight;
    boolean goingUp = false;
    boolean goingDown = false;
    GamepadEx driverGamepad;
    ButtonReader gamepadupButton,gamepadDownButton;
    int threshold;

    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        driverGamepad = new GamepadEx(gamepad1);
        gamepadupButton = new ButtonReader(driverGamepad, GamepadKeys.Button.Y);
        gamepadDownButton = new ButtonReader(driverGamepad, GamepadKeys.Button.DPAD_DOWN);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        backLeftWheel = hardwareMap.get(DcMotor.class, "Back_left_wheel");
        backRightWheel = hardwareMap.get(DcMotor.class, "Back_right_wheel");
        frontLeftWheel = hardwareMap.get(DcMotor.class, "Front_left_wheel");
        frontRightWheel = hardwareMap.get(DcMotor.class, "Front_right_wheel");
        CLAW = hardwareMap.servo.get("CLAW");
        linearLift = hardwareMap.get(DcMotorEx.class, "linearLift");
        linearLift2 = hardwareMap.get(DcMotorEx.class, "linearLift2");
        trayServoL = hardwareMap.get(Servo.class, "trayServoL");
        trayServoR = hardwareMap.get(Servo.class, "trayServoR");
        Distance = hardwareMap.get(DistanceSensor.class, "Distance");
        Touch = hardwareMap.get(DigitalChannel.class, "Touch");
        imu.initialize(parameters);

        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        //linearLift.setDirection(DcMotor.Direction.REVERSE);
        //linearLift2.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLift2.setTargetPositionTolerance(50);
        //linearLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //linearLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        linearLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //  linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status:", "initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            gamepadupButton.readValue();
            gamepadDownButton.readValue();

            /**if (gamepadupButton.wasJustPressed()){
                mockGoalLiftheight ++;
            } else if (gamepadDownButton.wasJustPressed()){
                mockGoalLiftheight -= 1;
            }*/
            if (gamepad2.dpad_up) {
                if (!upButton) {
                    //startPosition = linearLift.getCurrentPosition();
                    mockGoalLiftheight++;
                    upButton = true;

                } else {
                }

            } else {
                upButton = false;
            }

            if (gamepad2.dpad_down) {
                if (!downButton) {
                    // startPosition = linearLift.getCurrentPosition();
                    mockGoalLiftheight -= 1;
                    downButton = true;

                } else {
                }

            } else {
                downButton = false;
            }

            if (mockGoalLiftheight > 4) {
             mockGoalLiftheight = 4;
            } else if (mockGoalLiftheight < 0) {
                mockGoalLiftheight = 0;
            }

            /**switch (mockGoalLiftheight) {
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

                //default:
                  //  goalLiftHeight = 0;
                    //break;
            }*/

            //linearLift.setTargetPosition(goalLiftHeight);
            //linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (linearLift.getCurrentPosition() <= goalLiftHeight){
                linearLift.setPower(0);
                linearLift2.setPower(0);
            } else {
                linearLift.setTargetPosition(goalLiftHeight);
                linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearLift.setPower(.7);
            }

            if (mockGoalLiftheight == 0) {
                goalLiftHeight = 0;

            } else if (mockGoalLiftheight == 1) {
                goalLiftHeight = -560;

            } else if (mockGoalLiftheight == 2) {
                goalLiftHeight = -1120;

            } else if (mockGoalLiftheight == 3) {
                goalLiftHeight = -1680;

            } else if (mockGoalLiftheight == 4) {
                goalLiftHeight = -2240;

            }

           if (prevGoalLiftHeight > goalLiftHeight){
                goingUp = true;
                goingDown = false;
            } else if (prevGoalLiftHeight < goalLiftHeight){
                goingDown = true;
                goingUp = false;
            }

  /**          if (goingUp){
                while (linearLift.getCurrentPosition() > goalLiftHeight){
                    linearLift.setPower(-.7);
                    linearLift2.setPower(-.7);
                    telemetry.addLine("lifting");
                }


            } else if (goingDown){

                while (linearLift.getCurrentPosition() < goalLiftHeight){
                    linearLift.setPower(.7);
                    linearLift2.setPower(.7);
                }

            } else {
                linearLift.setPower(0);
                linearLift2.setPower(0);
            }

            /**while (linearLift.getCurrentPosition() != goalLiftHeight){

                linearLift.setTargetPosition(goalLiftHeight);
                linearLift2.setTargetPosition(goalLiftHeight);
                linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearLift.setPower(.1);
                linearLift.setPower(.1);

                telemetry.addLine("unbroken");

                if (linearLift.getCurrentPosition() == goalLiftHeight){
                    break;
                }

            }
            linearLift.setPower(0);
            linearLift2.setPower(0);

            telemetry.addLine("broken");



            /**linearLift.setTargetPosition(-1120);
             linearLift2.setTargetPosition(-1120);
             linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             linearLift.setPower(-.7);
             linearLift2.setPower(-.7);
             while (linearLift.isBusy()){

             }
             linearLift.setPower(0);
             linearLift2.setPower(0);
             */

                telemetry.addData("linear lift height:", linearLift.getCurrentPosition());
                telemetry.addData("linear lift 2 height:", linearLift2.getCurrentPosition());
                telemetry.addData("linear lift target height:", linearLift.getTargetPosition());
                telemetry.addData("linear lift 2 target height:", linearLift2.getTargetPosition());
                telemetry.addData("target linear lift height:", goalLiftHeight);
                telemetry.addData("mockGoalLiftHeight:", mockGoalLiftheight);
                telemetry.addData("button just pressed:",gamepadupButton.wasJustPressed());
                telemetry.update();

                prevGoalLiftHeight = goalLiftHeight;
        }
    }

}

