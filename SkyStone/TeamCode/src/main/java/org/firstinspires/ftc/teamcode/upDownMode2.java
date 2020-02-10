package org.firstinspires.ftc.teamcode;







import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;

import com.qualcomm.robotcore.hardware.Servo;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;







@TeleOp(name = "Skystone op mode",group = "competition modes")
@Disabled
public class upDownMode2 extends LinearOpMode {



    private Gyroscope imu;

    private DcMotorEx backLeftWheel, backRightWheel, frontLeftWheel, frontRightWheel;

    private DcMotor linearLift;

    private Servo CLAW;



    @Override

    public void runOpMode() throws InterruptedException {



        imu = hardwareMap.get(Gyroscope.class, "imu");

        backLeftWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Back_left_wheel");

        backRightWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Back_right_wheel");

        frontLeftWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Front_left_wheel");

        frontRightWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Front_right_wheel");

        linearLift = (DcMotor) hardwareMap.get(DcMotor.class, "linearLift");

        CLAW = hardwareMap.servo.get("CLAW");





        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        backRightWheel.setDirection(DcMotor.Direction.REVERSE);

        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);

        linearLift.setDirection(DcMotor.Direction.REVERSE);

        linearLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        telemetry.addData("Status", "Initialized");

        telemetry.update();



        waitForStart();



        double speed;

        double rotation;

        double strafe;

        double multiplier = 0;



        while (opModeIsActive()) {



            // Drive code



            speed = -this.gamepad1.left_stick_y * multiplier;

            rotation = this.gamepad1.left_stick_x * multiplier;

            strafe = -this.gamepad1.right_stick_x * multiplier;





            backLeftWheel.setPower(speed + strafe - rotation);

            backRightWheel.setPower(-speed + strafe - rotation);

            frontLeftWheel.setPower(speed + strafe + rotation);

            frontRightWheel.setPower(-speed + strafe + rotation);



            if (gamepad1.left_trigger > .5){

                multiplier = .4;

            } else{

                multiplier = 1;

            }



            // CLAAAAW code

            if (this.gamepad2.a) {

                CLAW.setPosition(0);

            } else if (this.gamepad2.y) {

                CLAW.setPosition(1);

            }





            int currentPosition = linearLift.getCurrentPosition();



            // You shoudln't be able to move the arm

            if (currentPosition >= 5 ) {

                if(gamepad2.left_trigger > .5) {

                    linearLift.setPower(-0.5);

                } else {

                    linearLift.setPower(0);

                }

            } else if (currentPosition < 0 ) {

                if(gamepad2.right_trigger > 0.5) {

                    linearLift.setPower(0.5);

                } else {

                    linearLift.setPower(0);

                }

            } else{

                if(gamepad2.right_trigger > 0.5) {

                    linearLift.setPower(0.5);

                } else if(gamepad2.left_trigger > 0.5) {

                    linearLift.setPower(-0.5);

                } else {

                    linearLift.setPower(0);

                }

            }





            telemetry.addData("CLAAAAAW position", CLAW.getPosition());

            telemetry.addData("Target Power", speed);

            telemetry.addData("Motor Power", backLeftWheel.getPower());

            telemetry.addData("Status", "Running");

            telemetry.addData ("Linear lift height", currentPosition);



            telemetry.update();



        }

    }

}