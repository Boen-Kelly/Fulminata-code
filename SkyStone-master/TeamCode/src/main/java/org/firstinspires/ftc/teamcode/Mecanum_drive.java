package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;


/**

 * Created by Boen on 10-10-19

 */

@TeleOp
@Disabled


public class Mecanum_drive extends LinearOpMode {

    private Gyroscope imu;

    private DcMotor Back_left_wheel;

    private DcMotor Back_right_wheel;

    private DcMotor Front_left_wheel;

    private DcMotor Front_right_wheel;

    private DcMotor Linear_lift;
    private Servo CLAW;


    @Override

    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(Gyroscope.class,"imu");

        Back_left_wheel = hardwareMap.get(DcMotor.class, "Back_left_wheel");

        Back_right_wheel = hardwareMap.get(DcMotor.class, "Back_right_wheel");

        Front_left_wheel = hardwareMap.get(DcMotor.class, "Front_left_wheel");

        Front_right_wheel = hardwareMap.get(DcMotor.class, "Front_right_wheel");

        Linear_lift = hardwareMap.get(DcMotor.class, "Right_linear_lift");
        CLAW = hardwareMap.servo.get("CLAW");

        Back_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        Back_right_wheel.setDirection(DcMotor.Direction.REVERSE);
        Front_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        Front_right_wheel.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");

        telemetry.update();

        //Wait for the game to start (driver presses PLAY)

        waitForStart();


        //run until the end of the match (driver presses STOP)

        double speed;

        double rotation;

        double strafe;

        double motorRotation = 0;


        boolean prevUpButton = false;

        boolean prevDownButton = false;


        while (opModeIsActive()) {


            speed = -this.gamepad1.left_stick_y;

            rotation = this.gamepad1.left_stick_x;

            strafe = -this.gamepad1.right_stick_x;


            Back_left_wheel.setPower(speed + strafe - rotation);

            Back_right_wheel.setPower(-speed + strafe - rotation);

            Front_right_wheel.setPower(-speed + strafe + rotation);

            Front_left_wheel.setPower(speed + strafe + rotation);

            telemetry.addData("CLAAAAAW position", CLAW.getPosition());
            telemetry.addData("Target Power", speed);
            telemetry.addData("Motor Power", Back_left_wheel.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();


            // If Up button was just pressed, add 50 to motor rotation

            //if (this.gamepad2.y) // && !prevUpButton)
            //{
                //motorRotation += 50;
                //Right_linear_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //Right_linear_lift.setTargetPosition((int) motorRotation);
                //Right_linear_lift.setPower(.5);
                //Right_linear_lift.setPower(0);
            }

            // If Down button was just pressed, subtract 50 to motor rotation

            //else if (this.gamepad2.a) //&& !prevDownButton)

            {
                //motorRotation -= 50;
                //Right_linear_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //Right_linear_lift.setTargetPosition((int) motorRotation);
                //Right_linear_lift.setPower(.5);
                //Right_linear_lift.setPower(0);
            }


            //Right_linear_lift.setTargetPosition((int) motorRotation);

            //Right_linear_lift.setPower(.5);

            //prevUpButton = this.gamepad2.y;

            //prevDownButton = this.gamepad2.a;

            //Right_linear_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (this.gamepad2.x) {

                CLAW.setPosition(0);

            }

            if (this.gamepad2.b) {

                CLAW.setPosition(1);
            }
            Linear_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            if (this.gamepad2.y) {

                // How many degrees per second should it go up

                //Linear_lift.setVelocity(60, AngleUnit.DEGREES);

            } else if (this.gamepad2.a) {

                // How many degrees per second should it go down

                //Linear_lift.setVelocity(-60, AngleUnit.DEGREES);

            } else {

                // Hold Current Position

                //Linear_lift.setVelocity(0, AngleUnit.DEGREES);

            }
        }

    }


