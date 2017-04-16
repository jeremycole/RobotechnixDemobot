package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutonomousBallSeeker")
@SuppressWarnings("unused")
public class AutonomousBallSeeker extends RobotechnixDemobotOpMode {
    private int[] irSeekerAngle_history;
    private int irSeekerAngle_index = 0;
    private int irSeekerAngle() {
        int angle = (int) robot.mIrSeekerSensor.getAngle();

        if (irSeekerAngle_history == null) {
            irSeekerAngle_history = new int[100];
            for (int i=0; i < irSeekerAngle_history.length; i++) {
                irSeekerAngle_history[i] = angle;
            }
        }

        irSeekerAngle_history[irSeekerAngle_index] = angle;
        irSeekerAngle_index = (irSeekerAngle_index+1) % irSeekerAngle_history.length;

        int sum = 0;
        for (int cur : irSeekerAngle_history) {
            sum += cur;
        }

        return sum / irSeekerAngle_history.length;
    }

    @Override
    public void robotRun() {
        while(shouldKeepRunning()) {
            int angle = irSeekerAngle();
            double strength = robot.mIrSeekerSensor.getStrength();

            telemetry.addData("angle", angle);
            telemetry.addData("strength", strength);
            telemetry.update();

            if (!robot.mIrSeekerSensor.signalDetected()) {
                robot.stop();
                continue;
            }

            if (Math.abs(angle) > 10) {
                RobotDrivetrain.RotationDirection direction;
                if (angle < 0)
                    direction = RobotDrivetrain.RotationDirection.LEFT;
                else
                    direction = RobotDrivetrain.RotationDirection.RIGHT;

                double speed = Math.min(0.4, 0.4 * (double) Math.abs(angle) / 100.0);
                speed = speed < 0.05 ? 0.0 : speed;
                robot.rotate(direction, speed);
            }

            if (strength < 0.5 && Math.abs(angle) < 20) {
                double speed = (0.5 - strength);
                speed = speed < 0.05 ? 0.0 : speed;
                robot.translate(0, speed);
            }
        }

        robot.stop();
    }
}
