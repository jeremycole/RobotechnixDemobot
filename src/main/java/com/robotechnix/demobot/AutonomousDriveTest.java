package com.robotechnix.demobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutonomousDriveTest")
@SuppressWarnings("unused")
public class AutonomousDriveTest extends RobotechnixDemobotOpMode {
    @Override
    public void robotRun() {
        //robot.translateDistance(0, 0.7, 100);
        //robot.rotateDegrees(RobotDrivetrain.RotationDirection.RIGHT, 0.8, 180);
        //robot.rotateDegrees(RobotDrivetrain.RotationDirection.LEFT, 0.8, 180);
        //robot.translateDistance(180, 0.2, 20);
        robot.translateToRangeGreaterThan(0, 0.6, 2.0, 100);
        robot.translateToRangeLessThan(180, 0.6, 1.0, 100);
        robot.translateToRangeGreaterThan(0, 0.6, 2.0, 100);
        robot.stop();
    }
}
