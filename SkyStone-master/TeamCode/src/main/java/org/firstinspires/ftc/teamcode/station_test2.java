package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp
//@Disabled
public class station_test2 extends LinearOpMode {

    private DcMotor backLeftWheel, backRightWheel, frontLeftWheel, frontRightWheel;
    private DcMotorEx  linearLift, linearLift2;
    private Servo CLAW, trayServoL, trayServoR;

    private GamepadEx driverGamepad, operatorGamepad;
    private ButtonReader upButton, downButton;
    private RevIMU gyro;

    DistanceSensor Distance;
    DigitalChannel Touch;
    private ElapsedTime runtime = new ElapsedTime();
    double startPosition;
    int mockGoalLiftHeight = 0;

    double liftMultiplier = 1;
    int goalLiftHeight = 0;
    int prevMockGoalLiftHeight;
    boolean goingUp = false;
    boolean goingDown = false;
    String loopIdentifier = "main loop";

    @Override
    public void runOpMode() throws InterruptedException {

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        upButton = new ButtonReader(operatorGamepad, GamepadKeys.Button.DPAD_UP);
        downButton = new ButtonReader(operatorGamepad, GamepadKeys.Button.DPAD_DOWN);

        gyro = new RevIMU(hardwareMap);
        gyro.init();

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

        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Touch.setMode(DigitalChannel.Mode.INPUT);

        linearLift.setTargetPositionTolerance(35);
        linearLift2.setTargetPositionTolerance(35);

        trayServoR.setPosition(1);
        trayServoL.setPosition(0);

        CLAW.setPosition(0);

        while ((((!gyro.getRevIMU().isGyroCalibrated()) && !isStopRequested()) && !linearLift.isMotorEnabled()) && !linearLift2.isMotorEnabled()) {

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

        while (opModeIsActive()) {
            upButton.readValue();
            downButton.readValue();

            if (upButton.wasJustPressed()) {
                mockGoalLiftHeight++;
            } else if (downButton.wasJustPressed()) {
                mockGoalLiftHeight -= 1;
            }

            if (mockGoalLiftHeight > 4) {
                mockGoalLiftHeight = 4;
            } else if (mockGoalLiftHeight < 0) {
                mockGoalLiftHeight = 0;
            }

            /**if (prevMockGoalLiftHeight > mockGoalLiftHeight){
                goingDown = true;
                goingUp = false;
            } else if (prevMockGoalLiftHeight < mockGoalLiftHeight){
                goingUp = true;
                goingDown = false;
            }*/

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

            if (mockGoalLiftHeight == 0) {

                linearLift.setTargetPosition(0);
                linearLift2.setTargetPosition(0);
                linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearLift.setPower(1);
                linearLift2.setPower(1);

            } else if (mockGoalLiftHeight == 1) {
                linearLift.setTargetPosition(-560);
                linearLift2.setTargetPosition(-560);
                linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearLift.setPower(1);
                linearLift2.setPower(1);
            } else if (mockGoalLiftHeight == 2) {
                linearLift.setTargetPosition(-1120);
                linearLift2.setTargetPosition(-1120);
                linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearLift.setPower(1);
                linearLift2.setPower(1);
            } else if (mockGoalLiftHeight == 3) {
                linearLift.setTargetPosition(-1680);
                linearLift2.setTargetPosition(-1680);
                linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearLift.setPower(1);
                linearLift2.setPower(1);
            } else if (mockGoalLiftHeight == 4) {
                linearLift.setTargetPosition(-2240);
                linearLift2.setTargetPosition(-2240);
                linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearLift.setPower(1);
                linearLift2.setPower(1);
            }

            /**if (goingUp && (linearLift.getCurrentPosition() <= goalLiftHeight)){
                linearLift.setPower(0);
                //linearLift2.setPower(0);
                loopIdentifier = "Stopped at up";
            } else {*/

                loopIdentifier = "Moving";
            //}

            //loopIdentifier = "main";


           /** else if (goingDown && (linearLift.getCurrentPosition() >= goalLiftHeight)){
                linearLift.setPower(0);
                linearLift2.setPower(0);
                loopIdentifier = "Stopped at down";
            }

           /** linearLift.setTargetPosition(goalLiftHeight);
            linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearLift2.setTargetPosition(goalLiftHeight);
            linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            linearLift.setPower(0.5);
            linearLift2.setPower(0.5);
            */


            telemetry.addData("linear lift height:", linearLift.getCurrentPosition());
            telemetry.addData("linear lift target height:", linearLift.getTargetPosition());
            telemetry.addData("linear lift 2 height:", linearLift2.getCurrentPosition());
            telemetry.addData("linear lift 2 target height:", linearLift2.getTargetPosition());
            telemetry.addData("target height:", goalLiftHeight);
            telemetry.addData("mockGoalLiftHeight:", mockGoalLiftHeight);
            telemetry.addData("button just pressed:",upButton.wasJustPressed());
            telemetry.addData("going up:", goingUp);
            telemetry.addData("going down:",goingDown);
            telemetry.addData("loop:",loopIdentifier);
            telemetry.update();


            prevMockGoalLiftHeight = mockGoalLiftHeight;
        }
    }

}