package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp (name = "linear lift reset")
public class Reset_Lift extends LinearOpMode {

    private DcMotor linearLift,linearLift2;
    double liftSlowdwnFactor;
    private GamepadEx operatorgamepad;
    private TriggerReader leftTrigger,rightTrigger;
    private ButtonReader x,y,a,b;

    @Override
    public void runOpMode()throws InterruptedException{

        linearLift = hardwareMap.get(DcMotor.class, "linearLift");
        linearLift2 = hardwareMap.get(DcMotor.class,"linearLift2");
        operatorgamepad = new GamepadEx(gamepad2);
        leftTrigger = new TriggerReader(operatorgamepad,GamepadKeys.Trigger.LEFT_TRIGGER);
        rightTrigger = new TriggerReader(operatorgamepad,GamepadKeys.Trigger.RIGHT_TRIGGER);
        x = new ButtonReader(operatorgamepad,GamepadKeys.Button.X);
        y = new ButtonReader(operatorgamepad,GamepadKeys.Button.Y);
        a = new ButtonReader(operatorgamepad,GamepadKeys.Button.A);
        b = new ButtonReader(operatorgamepad,GamepadKeys.Button.B);

        waitForStart();

        while (opModeIsActive()){

            rightTrigger.readValue();
            leftTrigger.readValue();


            linearLift.setPower(gamepad2.left_stick_y * liftSlowdwnFactor);
            linearLift2.setPower(gamepad2.left_stick_y * liftSlowdwnFactor);

            if (leftTrigger.isDown() || rightTrigger.isDown()){
                liftSlowdwnFactor = .5;
            } else if (leftTrigger.isDown() && rightTrigger.isDown()){
                liftSlowdwnFactor = .1;
            } else {
                liftSlowdwnFactor = 1;
            }


        }
    }

}
