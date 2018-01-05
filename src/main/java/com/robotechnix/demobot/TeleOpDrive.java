package com.robotechnix.demobot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "TeleOpDrive", group = "TeleOp")
@SuppressWarnings({"unused", "WeakerAccess"})
public class TeleOpDrive extends RobotechnixDemobotOpMode {
    private int mod(final int n, final int d) {
        return ((n % d) + d) % d;
    }

    private int getJoystickAngle(final double x, final double y) {
        return mod((int)Math.toDegrees(Math.atan2(x, -y)), 360);
    }

    private double getJoyStickMagnitude(final double x, final double y) {
        return Math.sqrt(Math.pow(Math.abs(x), 2) + Math.pow(Math.abs(y), 2));
    }

    @Override
    public void robotRun() {
        Telemetry.Item mTelemetryGyroItem = telemetry.addData("1. gyro", null);
        Telemetry.Item mTelemetryIrSeekerItem = telemetry.addData("2. ir", null);
        Telemetry.Item mTelemetryRangeItem = telemetry.addData("3. range", null);
        Telemetry.Item mTelemetryAngle = telemetry.addData("4. translation angle", null);
        Telemetry.Item mTelemetryPower = telemetry.addData("5. translation power", null);
        Telemetry.Item mTelemetryRotationPower = telemetry.addData("6. rotation power", null);

        while (shouldKeepRunning()) {
            double x1 = gamepad1.left_stick_x;
            double y1 = gamepad1.left_stick_y;
            double x2 = gamepad1.right_stick_x;
            double lt = gamepad1.left_trigger;
            double rt = gamepad1.right_trigger;

            robot.positionRangeServo((-lt + rt + 1.0) / 2.0);
            if(gamepad1.a)
                robot.positionClawServo(1.0);
            else
                robot.positionClawServo(0.0);

            if (x2 < 0)
                robot.rotate(RobotDrivetrain.RotationDirection.LEFT, -x2);
            else
                robot.rotate(RobotDrivetrain.RotationDirection.RIGHT, x2);

            int angle    = getJoystickAngle(x1, y1);
            double power = getJoyStickMagnitude(x1, y1);

            // Treat the angle as field-oriented.
            if (gamepad1.right_bumper)
                angle = mod(angle - robot.mGyroSensor.value().intValue(), 360);

            robot.translate(angle, power);

            mTelemetryGyroItem.setValue(robot.mGyroSensor.value());
            mTelemetryIrSeekerItem.setValue(robot.mIrSeekerAngleSensor.value());
            mTelemetryRangeItem.setValue(robot.mRangeSensor.value());
            mTelemetryAngle.setValue(angle);
            mTelemetryPower.setValue(power);
            mTelemetryRotationPower.setValue(x2);
            telemetry.update();
        }
    }
}
