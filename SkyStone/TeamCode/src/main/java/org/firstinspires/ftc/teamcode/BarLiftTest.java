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


import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;




/**
 * Created by Boen on 10-10-19
 */


@TeleOp(name = "girrafey test")


public class BarLiftTest extends LinearOpMode {

    private Gyroscope imu;

    private DcMotorEx backLeftWheel, backRightWheel, frontLeftWheel, frontRightWheel, linearLift, linearLift2;

    private Servo CLAW, trayServoL, trayServoR, Capstone;
    double startPosition;
    int goalLiftHeight = 0;
    double liftMultiplier = 1;
    boolean open;
    double totalTicks;
    private GamepadEx driverGamepad, operatorGamepad;
    private ButtonReader upButton, downButton, leftBumper, rightBumper;
    int mockGoalLiftHeight = 0;
    double speed;
    double rotation;
    double strafe;
    double multiplier = 0;
    boolean buttonPressed = false;
    double correctionMultiplier;
    private TriggerReader leftTrigger, rightTrigger;
    boolean openButton = false;
    private DcMotor barLift;

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
        barLift = hardwareMap.get(DcMotor.class,"barLift");
        //xbutton = new ButtonReader(operatorGamepad, GamepadKeys.Button.X);
        operatorGamepad = new GamepadEx(gamepad2);
        upButton = new ButtonReader(operatorGamepad, GamepadKeys.Button.DPAD_UP);
        downButton = new ButtonReader(operatorGamepad, GamepadKeys.Button.DPAD_DOWN);
        leftTrigger = new TriggerReader(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER);
        rightTrigger = new TriggerReader(operatorGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER);
        rightBumper = new ButtonReader(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        leftBumper = new ButtonReader(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER);


        linearLift.setTargetPositionTolerance(60);
        linearLift2.setTargetPositionTolerance(60);

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
        Capstone.setPosition(0);

        telemetry.addData("Status", "Initialized");

        telemetry.update();


        //Wait for the game to start (driver presses PLAY)


        waitForStart();
        while (opModeIsActive()){
            barLift.setPower(-gamepad2.left_stick_y);
        }

    }
}