package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class pidf_tuning extends LinearOpMode {
    DcMotorEx backLeftWheel,backRightWheel,frontLeftWheel,frontRightWheel;
    double currentVelocityBL,currentVelocityBR,currentVelocityFL,currentVelocityFR;
    double maxVelocityBL = 0.0;
    double maxVelocityBR = 0.0;
    double maxVelocityFL = 0.0;
    double maxVelocityFR = 0.0;

    @Override
    public void runOpMode() {
        backLeftWheel = hardwareMap.get(DcMotorEx.class, "Back_left_wheel");
        backRightWheel = hardwareMap.get(DcMotorEx.class, "Back_right_wheel");
        frontLeftWheel = hardwareMap.get(DcMotorEx.class, "Front_left_wheel");
        frontRightWheel = hardwareMap.get(DcMotorEx.class, "Front_right_wheel");

        //**
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         //*/

        /**
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         //*/

        backLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        //**
        backLeftWheel.setVelocityPIDFCoefficients(75,25,10,0);
        backRightWheel.setVelocityPIDFCoefficients(75,25,10,0);
        frontLeftWheel.setVelocityPIDFCoefficients(75,25,10,0);
        frontRightWheel.setVelocityPIDFCoefficients(75,25,10,0);
         //*/

        waitForStart();
        while (opModeIsActive()){

            /**
            backLeftWheel.setPower(1);
            backRightWheel.setPower(1);
            frontLeftWheel.setPower(1);
            frontRightWheel.setPower(1);
             //*/

            //**
            backLeftWheel.setVelocity(360, AngleUnit.DEGREES);
            backRightWheel.setVelocity(360,AngleUnit.DEGREES);
            frontLeftWheel.setVelocity(360,AngleUnit.DEGREES);
            frontRightWheel.setVelocity(360,AngleUnit.DEGREES);
             //*/

            currentVelocityBL = backLeftWheel.getVelocity();
            currentVelocityBR = backRightWheel.getVelocity();
            currentVelocityFL = frontLeftWheel.getVelocity();
            currentVelocityFR = frontLeftWheel.getVelocity();

            if (currentVelocityBL > maxVelocityBL) {
                maxVelocityBL = currentVelocityBL;
            }

            if (currentVelocityBR > maxVelocityBR) {
                maxVelocityBR = currentVelocityBR;
            }

            if (currentVelocityFL > maxVelocityFL) {
                maxVelocityFL = currentVelocityFL;
            }

            if (currentVelocityFR > maxVelocityFR) {
                maxVelocityFR = currentVelocityFR;
            }

            telemetry.addData("current velocity BL", currentVelocityBL);
            telemetry.addData("maximum velocity BL", maxVelocityBL);
            telemetry.addData("current velocity BR", currentVelocityBR);
            telemetry.addData("maximum velocity BR", maxVelocityBR);
            telemetry.addData("current velocity FL", currentVelocityFL);
            telemetry.addData("maximum velocity FL", maxVelocityFL);
            telemetry.addData("current velocity FR", currentVelocityFR);
            telemetry.addData("maximum velocity FR", maxVelocityFR);
            telemetry.update();
        }
    }
}