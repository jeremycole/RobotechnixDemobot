package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutonomousDriveTest")
@SuppressWarnings("unused")
public class AutonomousDriveTest extends RobotechnixDemobotOpMode {
    @Override
    public void robotRun() {
        robot.translateDistance(0, 0.5, 30);
        robot.stop();
    }
}
