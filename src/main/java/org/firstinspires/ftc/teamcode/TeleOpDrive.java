package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "TeleOpDrive", group = "TeleOp")
@SuppressWarnings({"unused", "WeakerAccess"})
public class TeleOpDrive extends RobotechnixDemobotOpMode {
    @Override
    public void robotRun() {
        double motorPower = 1.0;
        Telemetry.Item mTelemetryGyroItem = telemetry.addData("1. gyro", null);
        Telemetry.Item mTelemetryIrSeekerItem = telemetry.addData("2. ir", null);
        Telemetry.Item mTelemetryAngle = telemetry.addData("3. angle", null);
        Telemetry.Item mTelemetryPower = telemetry.addData("4. power", null);

        while (shouldKeepRunning()) {
            double x1 = gamepad1.left_stick_x;
            double y1 = gamepad1.left_stick_y;
            double x2 = gamepad1.right_stick_x;
            double lt = gamepad1.left_trigger;
            double rt = gamepad1.right_trigger;

            robot.positionRangeServo((-lt + rt + 1.0) / 2.0);

            if (x2 < 0)
                robot.rotate(RobotDrivetrain.RotationDirection.LEFT, Math.abs(x2));
            else
                robot.rotate(RobotDrivetrain.RotationDirection.RIGHT, Math.abs(x2));

            // Needs some tuning for direction, probably 90° off right now.
            int angle = ((((int)Math.toDegrees(Math.atan(y1/x1))) % 360) + 360) % 360;
            double power = Math.sqrt(Math.pow(Math.abs(x1), 2) + Math.pow(Math.abs(y1), 2));

            robot.translate(angle, power);

            mTelemetryGyroItem.setValue(robot.mGyroSensor.getData());
            mTelemetryIrSeekerItem.setValue(robot.mIrSeekerAngleSensor.getData());
            mTelemetryAngle.setValue(angle);
            mTelemetryPower.setValue(power);
            telemetry.update();
        }
    }
}
