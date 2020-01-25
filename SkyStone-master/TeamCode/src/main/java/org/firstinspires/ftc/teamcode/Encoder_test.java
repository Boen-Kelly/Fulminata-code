package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.Gyroscope;

import com.qualcomm.robotcore.hardware.Servo;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name = "Encoder test")
@Disabled
public class Encoder_test extends LinearOpMode {

    private DcMotorEx linearLift2;

    @Override

    public void runOpMode() throws InterruptedException {
        linearLift2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "linearLift2");

        linearLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("back left wheel tick count", linearLift2.getCurrentPosition());
            telemetry.update();
        }

    }


}
