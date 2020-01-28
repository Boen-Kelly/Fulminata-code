package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp (name = "linear lift reset")
public class Reset_Lift extends LinearOpMode {

    private DcMotor linearLift,linearLift2;

    @Override
    public void runOpMode()throws InterruptedException{

        linearLift = hardwareMap.get(DcMotor.class, "linearLift");
        linearLift2 = hardwareMap.get(DcMotor.class,"linearLift2");

        waitForStart();

        while (opModeIsActive()){


            linearLift.setPower(gamepad2.left_stick_y);
            linearLift2.setPower(gamepad1.left_stick_y);
        }
    }

}
