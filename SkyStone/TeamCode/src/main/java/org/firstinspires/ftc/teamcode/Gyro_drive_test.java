package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
public class Gyro_drive_test extends LinearOpMode {

    private DcMotor backLeftWheel, backRightWheel, frontLeftWheel, frontRightWheel, linearLift, linearLift2;
    private Servo CLAW, trayServoL, trayServoR;
    BNO055IMU imu;
    Orientation angles;
    DistanceSensor Distance;
    DigitalChannel Touch;
    private ElapsedTime runtime = new ElapsedTime();
    double straight,strafe,rotation;

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

        telemetry.addData("Status:", "initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            straight = gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            rotation = gamepad1.right_stick_x;

            if (gamepad1.right_trigger > .5) {
                if (((gamepad1.left_stick_y != 0) || (gamepad1.left_stick_x != 0)) || (gamepad1.right_stick_x != 0)) {
                    Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                }
            } else {
                frontRightWheel.setPower(straight - strafe - rotation);
                frontLeftWheel.setPower(straight + strafe + rotation);
                backRightWheel.setPower(straight + strafe - rotation);
                backLeftWheel.setPower(straight - strafe + rotation);

            }
        }

    }

    public void Drive(double straight,double strafe,double rotation)throws InterruptedException{

        double backleftSpeed,backrightSpeed,frontleftSpeed,frontrightSpeed;

        double frontRPower = straight - strafe - rotation;
        double frontLPower = straight + strafe + rotation;
        double backRPower = straight + strafe - rotation;
        double backLPower  = straight - strafe + rotation;
        //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //double target = angles.firstAngle;

    //    while ((((gamepad1.left_stick_y != 0) || (gamepad1.left_stick_x != 0)) || (gamepad1.right_stick_x != 0)) && opModeIsActive()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

          //  double correction = (target - currentHeading)/100;

            backleftSpeed = backLPower;
            backrightSpeed = backRPower;
            frontleftSpeed = frontLPower;
            frontrightSpeed = frontRPower;

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
        //}

        backLeftWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontRightWheel.setPower(0);


    }
}
