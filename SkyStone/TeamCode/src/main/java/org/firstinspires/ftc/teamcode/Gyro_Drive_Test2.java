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
@Disabled
public class Gyro_Drive_Test2 extends LinearOpMode {

    private DcMotor backLeftWheel, backRightWheel, frontLeftWheel, frontRightWheel;
    private DcMotorEx linearLift, linearLift2;
    private Servo CLAW, trayServoL, trayServoR;

    private GamepadEx driverGamepad, operatorGamepad;
    private ButtonReader upButton, downButton,xButton,aButton;
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
    private boolean arcadeMode = false;

    @Override
    public void runOpMode() throws InterruptedException {

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        upButton = new ButtonReader(operatorGamepad, GamepadKeys.Button.DPAD_UP);
        downButton = new ButtonReader(operatorGamepad, GamepadKeys.Button.DPAD_DOWN);
        xButton = new ButtonReader(driverGamepad, GamepadKeys.Button.X);
        aButton = new ButtonReader(driverGamepad, GamepadKeys.Button.A);

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

        while (opModeIsActive()){

            aButton.readValue();
            xButton.readValue();


            if (xButton.wasJustPressed()) {

            gyro.reset();

            }

            if (aButton.wasJustPressed()) {

                arcadeMode = !arcadeMode;

            }

            telemetry.addData("Arcade Mode (a)", arcadeMode);

            telemetry.addData("Heading (reset: x)", gyro.getHeading());

            telemetry.update();



            final double x = Math.pow(gamepad1.left_stick_x, 3.0);

            final double y = Math.pow(-gamepad1.left_stick_y, 3.0);



            final double rotation = Math.pow(gamepad1.right_stick_x, 3.0);

            final double direction = Math.atan2(x, y) + (arcadeMode ? gyro.getHeading() : 0.0);

            final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));



            final double lf = speed * Math.sin(direction + Math.PI / 4.0) + rotation;

            final double rf = speed * Math.cos(direction + Math.PI / 4.0) - rotation;

            final double lr = speed * Math.cos(direction + Math.PI / 4.0) + rotation;

            final double rr = speed * Math.sin(direction + Math.PI / 4.0) - rotation;


            backLeftWheel.setPower(lr);
            backRightWheel.setPower(rr);
            frontLeftWheel.setPower(lf);
            frontRightWheel.setPower(rf);
        }

    }
}
