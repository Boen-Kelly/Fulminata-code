package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//@Disabled
@Autonomous (name = "Testing mode")
@Disabled
public class testingMode extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeftWheel,backRightWheel,frontLeftWheel,frontRightWheel;
    BNO055IMU imu;


    @Override
    public void runOpMode() throws InterruptedException{
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        backLeftWheel =  hardwareMap.get(DcMotor.class, "Back_left_wheel");
        backRightWheel = hardwareMap.get(DcMotor.class,"Back_right_wheel");
        frontLeftWheel = hardwareMap.get(DcMotor.class,"Front_left_wheel");
        frontRightWheel = hardwareMap.get(DcMotor.class,"Front_right_wheel");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        waitForStart();

        backLeftWheel.setPower(1);



    }
    /**public void Drive(double rotation,double forward,double strafe){

        frontLeftWh.setPower(forward - strafe + rotation);
        Back_right_wheel.setPower(-forward - strafe + rotation);
        Front_right_wheel.setPower(-forward + strafe + rotation);
        Front_left_wheel.setPower(forward + strafe + rotation);
    }*/

}
