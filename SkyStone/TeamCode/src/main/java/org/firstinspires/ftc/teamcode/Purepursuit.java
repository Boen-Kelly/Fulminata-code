/**package org.firstinspires.ftc.teamcode;



import static org.firstinspires.ftc.teamcode.MathFunction.AngleWrap;
import static com.qualcomm.ftccommon.configuration.


public class Purepursuit {
    public static void goToPosition(double x, double y, double movementSpeed) {
        double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);

        double relativeAngleToPoint = AngleWrap (absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

    }
}*/
