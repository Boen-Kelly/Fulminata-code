package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "IMU Test")
@Disabled

public class imu_test extends LinearOpMode{


    BNO055IMU imu;
    private DcMotor backLeftWheel,backRightWheel,frontLeftWheel,frontRightWheel;
    Orientation angles;
    private ElapsedTime runtime = new ElapsedTime();


    @Override public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        backLeftWheel =  hardwareMap.get(DcMotor.class, "Back_left_wheel");
        backRightWheel = hardwareMap.get(DcMotor.class,"Back_right_wheel");
        frontLeftWheel = hardwareMap.get(DcMotor.class,"Front_left_wheel");
        frontRightWheel = hardwareMap.get(DcMotor.class,"Front_right_wheel");
        imu.initialize(parameters);
        sleep(1000);

        waitForStart();

        /**while (opModeIsActive()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading:", angles.firstAngle);
            telemetry.addData("Roll:", angles.secondAngle);
            telemetry.addData("Pitch:", angles.thirdAngle);
            telemetry.update();
        }*/

        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);


        turnRight(180);

        sleep(2000);

        turnLeft(0);





        runtime.reset();

        while (opModeIsActive()){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading:", -angles.firstAngle);
        telemetry.addData("Roll:", angles.secondAngle);
        telemetry.addData("Pitch:", angles.thirdAngle);
        telemetry.update();
        }


    }

    public void turnRight (double desiredHeading) {

        double speed;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startAngle = angles.firstAngle;


        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while ((Math.abs(angles.firstAngle) <= (desiredHeading - 2)) && !isStopRequested()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            double remainingAngle = -desiredHeading - angles.firstAngle;
            double dividingRatio = -desiredHeading - startAngle;

            speed = remainingAngle / dividingRatio;


            if (Math.abs(speed) < .2) {
                speed = .2;
            } else {
                speed = remainingAngle / dividingRatio;
            }

            backLeftWheel.setPower(speed);
            frontLeftWheel.setPower(speed);
            backRightWheel.setPower(-speed);
            frontRightWheel.setPower(-speed);


        }
        backLeftWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontRightWheel.setPower(0);

    }

     public void turnLeft (double desiredHeading){

        double speed;
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         double startAngle = angles.firstAngle;

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                while ((angles.firstAngle <= (desiredHeading -  2)) && !isStopRequested()){
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    double remainingAngle = desiredHeading - angles.firstAngle ;
                    double dividingRatio = desiredHeading - startAngle;

                    speed = remainingAngle / dividingRatio;

                    if (Math.abs(speed) < .2){
                        speed = .2;
                    }else {
                        speed = remainingAngle / dividingRatio;
                    }

                    backLeftWheel.setPower(-speed);
                    frontLeftWheel.setPower(-speed);
                    backRightWheel.setPower(speed);
                    frontRightWheel.setPower(speed);

                }
                backLeftWheel.setPower(0);
                frontLeftWheel.setPower(0);
                backRightWheel.setPower(0);
                frontRightWheel.setPower(0);

            }








    }

