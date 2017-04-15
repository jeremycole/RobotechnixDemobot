package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutonomousDriveTest")
@SuppressWarnings("unused")
public class AutonomousDriveTest extends RobotechnixDemobotOpMode {
    public void curveAndComeBack() {
        robot.translateDistance(0, 0.4, 30);
        robot.rotateToHeading(RobotDrivetrain.RotationDirection.RIGHT, 0.4, 90);
        robot.translateDistance(0, 0.4, 30);
        robot.rotateToHeading(RobotDrivetrain.RotationDirection.RIGHT, 0.4, 180);

        robot.rotateToHeading(RobotDrivetrain.RotationDirection.LEFT, 0.4, 90);
        robot.translateDistance(180, 0.4, 30);
        robot.rotateToHeading(RobotDrivetrain.RotationDirection.LEFT, 0.4, 0);
        robot.translateDistance(180, 0.4, 30);
    }
    @Override
    public void robotRun() {
        robot.translateDistance(0, 1.0, 60);
        robot.translateDistance(180, 1.0, 60);
        robot.stop();
    }
}
