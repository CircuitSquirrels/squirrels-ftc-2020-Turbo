package org.firstinspires.ftc.teamcode.DeadWheels;

import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.*;

public class OdometryConfig {

    public final double wheelDiameter = 60.0 * 0.0393701; // 60mm diameter
    public final double ticksPerRotation = 8192.0;
//    private final Navigation2D leftWheel = new Navigation2D(0,0,0);
//    private final Navigation2D centerWheel = new Navigation2D(0,0,0);
//    private final Navigation2D rightWheel = new Navigation2D(0,0,0);
    public double outerWheelDistance = 8.5;
    public double outerWheelXPos = -1.0;
    public double centerWheelXPos = -1.5;
    public double strafeErrorPerDegrees = centerWheelXPos * 2.0 * Math.PI / 360.0;

    public double inchesFromTicks(double ticks) {
        return ticks / ticksPerRotation * (wheelDiameter * Math.PI);
    }
}
