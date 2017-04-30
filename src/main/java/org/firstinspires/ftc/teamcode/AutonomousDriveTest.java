package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutonomousDriveTest")
@SuppressWarnings("unused")
public class AutonomousDriveTest extends RobotechnixDemobotOpMode {
    @Override
    public void robotRun() {
        robot.translateDistance(0, 0.2, 20);
        robot.rotateDegrees(RobotDrivetrain.RotationDirection.RIGHT, 0.4, 180);
        robot.rotateDegrees(RobotDrivetrain.RotationDirection.LEFT, 0.4, 180);
        robot.translateDistance(180, 0.2, 20);
        robot.stop();
    }
}
