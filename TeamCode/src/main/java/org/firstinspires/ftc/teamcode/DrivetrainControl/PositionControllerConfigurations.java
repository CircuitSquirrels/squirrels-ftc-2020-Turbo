package org.firstinspires.ftc.teamcode.DrivetrainControl;

import org.firstinspires.ftc.teamcode.DrivetrainControl.PositionController.PositionControllerConfiguration;
import org.firstinspires.ftc.teamcode.miniPID.MiniPID.MiniPIDConfiguration;
import org.firstinspires.ftc.teamcode.miniPID.MiniPID;

public class PositionControllerConfigurations {

    // This configuration is like the one tested on the first day, which worked out of the box.
    //MiniPIDConfiguration(double P, double I, double D, double maxOutput, double maxIOutput, double rampRate, double positionTolerance) {
    public PositionControllerConfiguration firstFunctioning = new PositionControllerConfiguration(
            new MiniPIDConfiguration(0.1,0.0,0.1,1.0,0.3,3.0,0.5), // Linear
            new MiniPIDConfiguration(0.7,0.0,0.3,0.7,0.3,3.0,Math.toRadians(5.0)) // Rotation (radians)
    );


}
