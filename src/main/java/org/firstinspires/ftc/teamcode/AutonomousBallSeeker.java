package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Timer;
import java.util.TimerTask;

@Autonomous(name = "AutonomousBallSeeker")
@SuppressWarnings("unused")
public class AutonomousBallSeeker extends RobotechnixDemobotOpMode {
    static double targetRangeToBall = 0.5;
    static double maxRotationSpeed = 0.4;
    static double maxTranslationSpeed = 0.8;

    public void followTheBall() {
        double angle    = robot.mIrSeekerAngleSensor.getData();
        double strength = robot.mIrSeekerStrengthSensor.getData();

        telemetry.addData("angle", angle);
        telemetry.addData("strength", strength);
        telemetry.update();

        if (!robot.mRawIrSeekerSensor.signalDetected() || strength < 0.03) {
            robot.stop();
            return;
        }

        if (Math.abs(angle) > 3) {
            RobotDrivetrain.RotationDirection direction;
            if (angle < 0)
                direction = RobotDrivetrain.RotationDirection.LEFT;
            else
                direction = RobotDrivetrain.RotationDirection.RIGHT;

            double speed = Math.min(maxRotationSpeed,
                    maxRotationSpeed * ((double) Math.abs(angle) / 100.0));
            //speed = speed < 0.10 ? 0.0 : speed;
            robot.rotate(direction, speed);
        } else {
            robot.rotate(null, 0.0);
        }

        if (strength < targetRangeToBall) {
            double speed = (targetRangeToBall - strength) / targetRangeToBall;
            speed *= maxTranslationSpeed;
            //speed = speed < 0.02 ? 0.0 : speed;
            robot.translate(0, speed);
        } else {
            robot.translate(0, 0.0);
        }
    }

    Timer mTimer;
    TimerTask mTimerTask;

    @Override
    public void robotRun() {
        mTimer = new Timer();

        mTimerTask = new TimerTask() {
            @Override
            public void run() {
                followTheBall();
            }
        };

        mTimer.schedule(mTimerTask, 0, 1);

        while(shouldKeepRunning()) {
            idle();
        }
    }

    @Override
    public void robotStop() {
        mTimer.cancel();
        robot.stop();
    }
}
