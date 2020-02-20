package org.firstinspires.ftc.teamcode;





import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



import com.qualcomm.robotcore.hardware.DcMotor;



import com.qualcomm.robotcore.hardware.DcMotorEx;



import com.qualcomm.robotcore.hardware.Gyroscope;



import com.qualcomm.robotcore.hardware.Servo;



import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;





import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;









/**

 * Created by Boen on 10-10-19

 */





@TeleOp(name = "Skystone op mode OLD", group = "competition modes")





public class upDownModeOLD extends LinearOpMode {



    private Gyroscope imu;



    private DcMotorEx backLeftWheel, backRightWheel, frontLeftWheel, frontRightWheel, linearLift, linearLift2;



    private Servo CLAW, trayServoL, trayServoR,Capstone;

    double startPosition;

    int goalLiftHeight = 0;

    boolean upButton;

    boolean downButton;

    double liftMultiplier = 1;

    boolean open;

    double totalTicks;
    boolean openButton = false;
    double strafeMultiplier = 1;
    double strafeMultiplierslowdown = 1;





    @Override





    public void runOpMode() throws InterruptedException {



        imu = hardwareMap.get(Gyroscope.class, "imu");

        backLeftWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Back_left_wheel");

        backRightWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Back_right_wheel");

        frontLeftWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Front_left_wheel");

        frontRightWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Front_right_wheel");

        linearLift = (DcMotorEx) hardwareMap.get(DcMotor.class, "linearLift");

        linearLift2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "linearLift2");

        CLAW = hardwareMap.servo.get("CLAW");

        trayServoL = hardwareMap.servo.get("trayServoL");

        trayServoR = hardwareMap.servo.get("trayServoR");

        Capstone = hardwareMap.get(Servo.class, "Capstone");





//        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        //backRightWheel.setDirection(DcMotor.Direction.REVERSE);



        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);



        //frontRightWheel.setDirection(DcMotor.Direction.REVERSE);



        linearLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



        linearLift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



        linearLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        linearLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        CLAW.setPosition(.65);

        trayServoL.setPosition(0);

        trayServoR.setPosition(1);

        Capstone.setPosition(1);



        telemetry.addData("Status", "Initialized");



        telemetry.update();





        //Wait for the game to start (driver presses PLAY)





        waitForStart();





        //run until the end of the match (driver presses STOP)





        double speed;

        double rotation;

        double strafe;

        double multiplier = 0;



        while (opModeIsActive()) {



            // Drive code




            speed = (this.gamepad1.left_stick_y * multiplier) * 540;



            rotation = (-this.gamepad1.left_stick_x * multiplier) * 540;



            strafe = (-this.gamepad1.right_stick_x * multiplier) * 540;



            //double backLeft

            backLeftWheel.setVelocity((speed + strafe - rotation));



            backRightWheel.setVelocity((-speed + strafe - rotation));



            frontLeftWheel.setVelocity((speed + strafe + rotation));



            frontRightWheel.setVelocity((-speed + strafe + rotation));



            if (gamepad1.left_trigger > .5) {

                multiplier = .4;

            } else {

                multiplier = 1;

            }





            // CLAAAAW code

            if (this.gamepad2.a) {



                CLAW.setPosition(1);



            } else if (this.gamepad2.y) {



                CLAW.setPosition(0);



            }



            if(this.gamepad1.left_bumper){

                trayServoL.setPosition(1);

                trayServoR.setPosition(0);

            }else{

                trayServoL.setPosition(0);

                trayServoR.setPosition(1);

            }



            //**

            //This code is for slowing down the lift, if we don't use it, then just un-comment out this code ^^ above



            if (gamepad2.right_trigger > .25) {

                liftMultiplier = .5;

            } else {

                liftMultiplier = 1;

            }

            //*/







            int currentPosition = linearLift.getCurrentPosition();

            double stickHeight = this.gamepad2.left_stick_y;





            if (currentPosition <= -4900) {

                if (stickHeight > .25) {

                    linearLift.setPower(stickHeight /** liftMultiplier*/);

                    linearLift2.setPower(stickHeight /** liftMultiplier*/);

                } else {

                    linearLift.setPower(0);

                    linearLift2.setPower(0);

                }

            } else if (currentPosition >= 0) {

                if (stickHeight < -.25) {

                    linearLift.setPower(stickHeight /** liftMultiplier*/);

                    linearLift2.setPower(stickHeight /** liftMultiplier*/);

                } else {

                    linearLift.setPower(0);

                    linearLift2.setPower(0);

                }

            } else {

                if ((stickHeight > .25) /**&& !(currentPosition >= 0)*/) {

                    linearLift.setPower(stickHeight /** liftMultiplier*/);

                    linearLift2.setPower(stickHeight /** liftMultiplier*/);

                } else if ((stickHeight < -0.25) /**&& !(currentPosition <= -1600)*/) {

                    linearLift.setPower(stickHeight/** * liftMultiplier*/);

                    linearLift2.setPower(stickHeight/** * liftMultiplier*/);

                } else {

                    linearLift.setPower(0);

                    linearLift2.setPower(0);

                }

            }



           /** if ((this.gamepad2.left_trigger > .5) && (gamepad2.x)) {

                if (!upButton) {



                    open = !open;

                    upButton = true;



                } else {

                }



            } else {

                upButton = false;

            }*/



            /**if (((gamepad2.left_trigger > .5) && gamepad2.x)){

                if (open){

                    Capstone.setPosition(1);

                } else if (open == false){

                    Capstone.setPosition(0);

                }



            }else {

                Capstone.setPosition(0);

            }*/



            if (goalLiftHeight > 4) {

                goalLiftHeight = 3;

            }

            if (goalLiftHeight < 0) {

                goalLiftHeight = 0;

            }



            if (gamepad2.dpad_up) {

                if (!upButton) {

                    startPosition = linearLift.getCurrentPosition();

                    goalLiftHeight =+ 240;

                    upButton = true;



                } else {

                }



            } else {

                upButton = false;

            }



            if (gamepad2.dpad_down) {

                if (!downButton) {

                    startPosition = linearLift.getCurrentPosition();

                    goalLiftHeight =- 240;

                    downButton = true;



                } else {

                }



            } else {

                downButton = false;

            }



            /**if (this.gamepad2.dpad_up) {

             if (goalLiftHeight == 1) {

             linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift.setTargetPosition(1120);

             linearLift2.setTargetPosition(1120);

             linearLift.setPower(.5);

             linearLift2.setPower(.5);



             while (linearLift.isBusy() && linearLift2.isBusy()){



             }



             } else if (goalLiftHeight == 2) {

             linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift.setTargetPosition(1120);

             linearLift2.setTargetPosition(1120);

             linearLift.setPower(.5);

             linearLift2.setPower(.5);



             while (linearLift.isBusy() && linearLift2.isBusy()){



             }

             } else if (goalLiftHeight == 3) {

             linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift.setTargetPosition(1120);

             linearLift2.setTargetPosition(1120);

             linearLift.setPower(.5);

             linearLift2.setPower(.5);

             while (linearLift.isBusy() && linearLift2.isBusy()){



             }

             } else if (goalLiftHeight == 4) {

             linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift.setTargetPosition(1120);

             linearLift2.setTargetPosition(1120);

             linearLift.setPower(.5);

             linearLift2.setPower(.5);

             while (linearLift.isBusy() && linearLift2.isBusy()){



             }

             }

             } else if (this.gamepad2.dpad_down) {

             if (goalLiftHeight == 0) {

             linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift.setTargetPosition(1120);

             linearLift2.setTargetPosition(1120);

             linearLift.setPower(.5);

             linearLift2.setPower(.5);

             while (linearLift.isBusy() && linearLift2.isBusy()){



             }

             } else if (goalLiftHeight == 1) {

             linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift.setTargetPosition(1120);

             linearLift2.setTargetPosition(1120);

             linearLift.setPower(.5);

             linearLift2.setPower(.5);

             while (linearLift.isBusy() && linearLift2.isBusy()){



             }

             } else if (goalLiftHeight == 2) {

             linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift.setTargetPosition(1120);

             linearLift2.setTargetPosition(1120);

             linearLift.setPower(.5);

             linearLift2.setPower(.5);

             while (linearLift.isBusy() && linearLift2.isBusy()){



             }

             } else if (goalLiftHeight == 3) {

             linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift.setTargetPosition(1120);

             linearLift2.setTargetPosition(1120);

             linearLift.setPower(.5);

             linearLift2.setPower(.5);

             while (linearLift.isBusy() && linearLift2.isBusy()){



             }

             }

             } else {

             linearLift.setPower(0);

             linearLift2.setPower(0);

             }

             double ticksLeft = goalLiftHeight - linearLift.getCurrentPosition();

             totalTicks = goalLiftHeight - startPosition;

             double approachSpeed = ticksLeft / totalTicks;



             if (linearLift.getCurrentPosition() <= goalLiftHeight){

             linearLift.setTargetPosition(goalLiftHeight);

             linearLift2.setTargetPosition(goalLiftHeight);

             linearLift.setPower(approachSpeed);

             linearLift2.setPower(approachSpeed);

             linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             }else if (linearLift.getCurrentPosition() >= goalLiftHeight){

             linearLift.setTargetPosition(goalLiftHeight);

             linearLift2.setTargetPosition(goalLiftHeight);

             linearLift.setPower(approachSpeed);

             linearLift2.setPower(approachSpeed);

             linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             linearLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             }*/





            if (gamepad2.left_stick_button) {

                linearLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                linearLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            } else {

                linearLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                linearLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



            }

            if (gamepad2.left_bumper && (gamepad2.x)) {

                if (!openButton) {



                    open = !open;

                    openButton = true;



                } else {

                }



            } else {

                openButton = false;

            }


            if (gamepad2.left_bumper && this.gamepad2.x) {
                if (open) {
                    Capstone.setPosition(0);
                } else if (open == false) {
                    Capstone.setPosition(1);
                }
            }

            /**if ((Math.abs(strafe) > .01)){

                strafeMultiplier = 5;
                strafeMultiplierslowdown = .5;

            } else {
                strafeMultiplier = 1;
                strafeMultiplierslowdown = 1;
            }*/




                telemetry.addData("CLAAAAAW position", CLAW.getPosition());

            telemetry.addData("Target Power", speed);

            telemetry.addData("Motor Power", backLeftWheel.getPower());

            telemetry.addData("Status", "Running");

            telemetry.addData("Linear lift height", linearLift.getCurrentPosition());
            /**telemetry.addData("back left wheel power:",backLeftWheel.getPower());
            telemetry.addData("back right wheel power:",backRightWheel.getPower());
            telemetry.addData("front left wheel power:",frontLeftWheel.getPower());
            telemetry.addData("front Right wheel power:",frontRightWheel.getPower());
            telemetry.addData("strafe multiplier:", strafeMultiplier);
            telemetry.addData("strafe multiplier slowdown:",strafeMultiplierslowdown);
*/


            telemetry.update();





        }





    }



}