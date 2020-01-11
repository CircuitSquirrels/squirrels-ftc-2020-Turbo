package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class PositionController {

    private RobotHardware opMode;
    private MecanumNavigation mecanumNavigation;

    public PositionController(RobotHardware opMode, MecanumNavigation mecanumNavigation) {
        this.opMode = opMode;
        this.mecanumNavigation = mecanumNavigation;
    }


}
