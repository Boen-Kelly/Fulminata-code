package org.firstinspires.ftc.teamcode;







import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.Gyroscope;

import com.qualcomm.robotcore.hardware.Servo;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;







@TeleOp(name = "Lift_test",group = "test modes")
@Disabled
public class upDownMode3 extends LinearOpMode {



    private Gyroscope imu;

    private DcMotor linearLift;





    @Override

    public void runOpMode() throws InterruptedException {





        linearLift = (DcMotor) hardwareMap.get(DcMotor.class, "linearLift");



        linearLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        telemetry.addData("Status", "Initialized");

        telemetry.update();



        waitForStart();



        while (opModeIsActive()) {



           // int currentPosition = linearLift.getCurrentPosition();



            // You shoudln't be able to move the arm

            /*if (currentPosition >= 5 ) {

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

            }*/



            telemetry.addData ("Linear lift height", linearLift.getCurrentPosition());



            telemetry.update();



        }

    }

}