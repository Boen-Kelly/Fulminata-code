package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Disabled
public class Distance_sensor_test extends LinearOpMode {

    DistanceSensor Distance;

    @Override public void runOpMode() {

        Distance = hardwareMap.get(DistanceSensor.class,"Distance");

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Distance in inches", Distance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }

}
