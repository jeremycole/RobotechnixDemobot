package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutonomousDriveTest")
@SuppressWarnings("unused")
public class AutonomousDriveTest extends RobotechnixDemobotOpMode {
    @Override
    public void robotRun() {
        robot.translate(0, 0.2);
        while(shouldKeepRunning()) {
            idle();
        }
        robot.stop();
    }
}
