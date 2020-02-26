package org.firstinspires.ftc.teamcode.deadWheels;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Frame2D;

public interface Localizer {

    void update(RobotHardware robotHardware);

    Navigation2D getCurrentPosition();

    void setCurrentPosition(Navigation2D currentPosition);

    Frame2D getRobotFrame();

}


