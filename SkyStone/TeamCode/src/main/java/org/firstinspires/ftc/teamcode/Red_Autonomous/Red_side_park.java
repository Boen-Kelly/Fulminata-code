package org.firstinspires.ftc.teamcode.Red_Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous (name = "Red side parking, start on right", group = "Red")
@Disabled
public class Red_side_park extends LinearOpMode {

    private DcMotor backLeftWheel,backRightWheel,frontLeftWheel,frontRightWheel,linearLift,linearLift2;
    private Servo CLAW;
    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode()throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        backLeftWheel =  hardwareMap.get(DcMotor.class, "Back_left_wheel");
        backRightWheel = hardwareMap.get(DcMotor.class,"Back_right_wheel");
        frontLeftWheel = hardwareMap.get(DcMotor.class,"Front_left_wheel");
        frontRightWheel = hardwareMap.get(DcMotor.class,"Front_right_wheel");
        CLAW = hardwareMap.servo.get("CLAW");
        linearLift = hardwareMap.get(DcMotor.class,"linearLift");
        linearLift2 = hardwareMap.get(DcMotor.class, "linearLift2");
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

        CLAW.setPosition(0);

        while((!imu.isGyroCalibrated()) && !isStopRequested()){

        }

        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status:","initialized");
        telemetry.update();

        waitForStart();

        Lift(1120,1);

        driveStraight(750,.7,0);

        Drop(0,.5);
        CLAW.setPosition(1);
        sleep(500);
        Lift(1120,1);

        backup(-250,-.4,0);
        Drop(140,.5);
        CLAW.setPosition(0);

        driveStraight(500,0,-.7);

    }
    public void Drive(double rotation,double forward,double strafe){

        backLeftWheel.setPower(forward + rotation - strafe);
        backRightWheel.setPower(forward - rotation + strafe);
        frontLeftWheel.setPower(forward + rotation + strafe);
        frontRightWheel.setPower(forward - rotation - strafe);
    }

    public void driveStraight(int distance, double straight,double strafe)throws InterruptedException{

        double backleftSpeed,backrightSpeed,frontleftSpeed,frontrightSpeed;


        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target = angles.firstAngle;
        double startPosition = backLeftWheel.getCurrentPosition();
        double frontRPower = straight - strafe;
        double frontLPower = straight + strafe;
        double backRPower = straight + strafe;
        double backLPower  = straight - strafe;

        while (((backLeftWheel.getCurrentPosition() < (distance + startPosition)) && (backRightWheel.getCurrentPosition() < (distance + startPosition))) && !isStopRequested()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

            double correction = (target - currentHeading)/100;

            backleftSpeed = backLPower - (correction * 2);
            backrightSpeed = backRPower + (correction * 2);
            frontleftSpeed = frontLPower - (correction * 2);
            frontrightSpeed = frontRPower + (correction * 2);

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
            telemetry.addData("desired heading:", startPosition);
        }

        backLeftWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontRightWheel.setPower(0);


    }

    public void Lift (int Height, double power)throws InterruptedException{
        while((linearLift.getCurrentPosition() >= -Height) && !isStopRequested()){
            linearLift.setPower(-power);
            linearLift2.setPower(-power);
        }
        linearLift.setPower(0);
        linearLift2.setPower(0);
    }

    public void Drop (int Height, double power)throws InterruptedException{
        while((linearLift.getCurrentPosition() <= -Height) && !isStopRequested()){
            linearLift.setPower(power);
            linearLift2.setPower(power);
        }
        linearLift.setPower(0);
        linearLift2.setPower(0);
    }

    public void backup (int distance, double straight,double strafe)throws InterruptedException{

        double backleftSpeed,backrightSpeed,frontleftSpeed,frontrightSpeed;


        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target = angles.firstAngle;
        double startPosition = backLeftWheel.getCurrentPosition();
        double frontRPower = straight - strafe;
        double frontLPower = straight + strafe;
        double backRPower = straight + strafe;
        double backLPower  = straight - strafe;

        while (((backLeftWheel.getCurrentPosition() > (distance + startPosition)) && (backRightWheel.getCurrentPosition() > (distance + startPosition))) && !isStopRequested()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

            double correction = (target - currentHeading)/100;

            backleftSpeed = backLPower - (correction * 2);
            backrightSpeed = backRPower + (correction * 2);
            frontleftSpeed = frontLPower - (correction * 2);
            frontrightSpeed = frontRPower + (correction * 2);

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
            telemetry.addData("desired heading:", startPosition);
        }

        backLeftWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontRightWheel.setPower(0);


    }
}
