package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "linear lift test")
@Disabled


public class linear_lift_test extends LinearOpMode {


    private DcMotor linearLift, linearLift2;
    Servo CLAW;


    @Override

    public void runOpMode() throws InterruptedException{

        linearLift = (DcMotor) hardwareMap.get(DcMotor.class, "linearLift");
        linearLift2 = hardwareMap.get(DcMotor.class,"linearLift2");
        CLAW = hardwareMap.get(Servo.class,"CLAW");

        linearLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //linearLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status:","Initialized");
        telemetry.update();
        waitForStart();

            CLAW.setPosition(0);

            Lift(1120,1);
            Drop(0,.5);


            telemetry.addData("linear lift height:", linearLift.getCurrentPosition());
            telemetry.update();

            sleep(500000);

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
}
