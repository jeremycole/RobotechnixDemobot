package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import java.util.HashMap;
import java.util.Locale;

public class RobotechnixDemobot {
    private interface DistanceFromTarget {
        double distance();
    }

    final private static double DRIVE_SLIPPAGE_FACTOR  = 1.00;
    final private static double STRAFE_SLIPPAGE_FACTOR = 1.08;
    final private static double ROTATE_SLIPPAGE_FACTOR = 1.00;

    LinearOpMode mOpMode;
    RobotDrivetrain mDrivetrain;
    GyroSensor mGyroSensor;

    final public static String LOG_TAG = "RobotechnixDemobot";
    public void info(String msg) {
        Log.i(LOG_TAG, msg);
    }

    RobotechnixDemobot(LinearOpMode opmode) {
        mOpMode = opmode;
    }

    // If stop was requested, throw a StopImmediatelyException which will be
    // caught by runOpMode to stop the robot immediately.
    public boolean shouldKeepRunning() {
        if(mOpMode.isStarted() && mOpMode.isStopRequested())
            throw new StopImmediatelyException();
        return true;
    }


    private void initializeDrivetrain() {
        mDrivetrain = new RobotDrivetrain();
        mDrivetrain.setMaxSpeed(2500);
        mDrivetrain.setEncoderCountsPerRevolution(1120);
        mDrivetrain.setGearRatio(1.0);
        mDrivetrain.setWheelDiameter(10.0);

        final RobotMotor mFL = new RobotMotor("mFL",
                mOpMode.hardwareMap.dcMotor.get("mFL"),
                DcMotor.Direction.FORWARD);
        mDrivetrain.addRobotMotor(mFL);

        final RobotMotor mFR = new RobotMotor("mFR",
                mOpMode.hardwareMap.dcMotor.get("mFR"),
                DcMotor.Direction.REVERSE);
        mDrivetrain.addRobotMotor(mFR);

        final RobotMotor mBL = new RobotMotor("mBL",
                mOpMode.hardwareMap.dcMotor.get("mBL"),
                DcMotor.Direction.FORWARD);
        mDrivetrain.addRobotMotor(mBL);

        final RobotMotor mBR = new RobotMotor("mBR",
                mOpMode.hardwareMap.dcMotor.get("mBR"),
                DcMotor.Direction.REVERSE);
        mDrivetrain.addRobotMotor(mBR);

        final RobotDrivetrain.DriveParametersFactory drive =
                new RobotDrivetrain.DriveParametersFactory(1.0, DRIVE_SLIPPAGE_FACTOR);

        final RobotDrivetrain.DriveParametersFactory strafe =
                new RobotDrivetrain.DriveParametersFactory(1.0, STRAFE_SLIPPAGE_FACTOR);

        final RobotDrivetrain.DriveParametersFactory rotate =
                new RobotDrivetrain.DriveParametersFactory(1.0, ROTATE_SLIPPAGE_FACTOR);

        mDrivetrain.addTranslation(new RobotDrivetrain.Translation(0,
                new HashMap<RobotMotor, RobotDrivetrain.DriveParameters>() {{
                    put(mFL, drive.power(+1.0));
                    put(mFR, drive.power(+1.0));
                    put(mBL, drive.power(+1.0));
                    put(mBR, drive.power(+1.0));
                }}));

        mDrivetrain.addTranslation(new RobotDrivetrain.Translation(90,
                new HashMap<RobotMotor, RobotDrivetrain.DriveParameters>() {{
                    put(mFL, strafe.power(+1.0));
                    put(mFR, strafe.power(-1.0));
                    put(mBL, strafe.power(-1.0));
                    put(mBR, strafe.power(+1.0));
                }}));

        mDrivetrain.addTranslation(new RobotDrivetrain.Translation(180,
                new HashMap<RobotMotor, RobotDrivetrain.DriveParameters>() {{
                    put(mFL, drive.power(-1.0));
                    put(mFR, drive.power(-1.0));
                    put(mBL, drive.power(-1.0));
                    put(mBR, drive.power(-1.0));
                }}));

        mDrivetrain.addTranslation(new RobotDrivetrain.Translation(270,
                new HashMap<RobotMotor, RobotDrivetrain.DriveParameters>() {{
                    put(mFL, strafe.power(-1.0));
                    put(mFR, strafe.power(+1.0));
                    put(mBL, strafe.power(+1.0));
                    put(mBR, strafe.power(-1.0));
                }}));

        mDrivetrain.addRotation(new RobotDrivetrain.Rotation(
                RobotDrivetrain.RotationDirection.LEFT,
                new HashMap<RobotMotor, RobotDrivetrain.DriveParameters>() {{
                    put(mFL, rotate.power(-1.0));
                    put(mFR, rotate.power(+1.0));
                    put(mBL, rotate.power(-1.0));
                    put(mBR, rotate.power(+1.0));
                }}));

        mDrivetrain.addRotation(new RobotDrivetrain.Rotation(
                RobotDrivetrain.RotationDirection.RIGHT,
                new HashMap<RobotMotor, RobotDrivetrain.DriveParameters>() {{
                    put(mFL, rotate.power(+1.0));
                    put(mFR, rotate.power(-1.0));
                    put(mBL, rotate.power(+1.0));
                    put(mBR, rotate.power(-1.0));
                }}));
    }

    private void initializeSensors() {
        mGyroSensor = mOpMode.hardwareMap.gyroSensor.get("gyro");
        mGyroSensor.calibrate();
        mOpMode.telemetry.addData(">", "Calibrating gyro...");
        mOpMode.telemetry.update();
        while (mGyroSensor.isCalibrating() && shouldKeepRunning());
    }

    public void initialize() {
        initializeDrivetrain();
        initializeSensors();

        mOpMode.telemetry.addData(">", "Ready.");
        mOpMode.telemetry.update();
    }

    public void stop() {
        info("stop");
        mDrivetrain.stop();
    }

    private void waitForTarget(DistanceFromTarget distanceFromTarget) {
        double distance;
        do {
            distance = distanceFromTarget.distance();
            //info(String.format(Locale.US, "distance=%.2f", distance));
            if (distance < 1.0)
                mDrivetrain.adjustSpeed(distance);
        } while (shouldKeepRunning() && distance > 0.0);
    }

    public void translate(int angle, double speed) {
        info(String.format(Locale.US, "translate(angle=%d, speed=%.2f)", angle, speed));
        mDrivetrain.translate(angle, speed);
    }

    class AnyMotorReachedTarget implements DistanceFromTarget {
        @Override
        public double distance() {
            return mDrivetrain.minDistanceUntilAnyEncoderSatisfied();
        }
    }

    public void translateDistance(int angle, double speed, double distance) {
        info(String.format(Locale.US, "translateDistance(angle=%d, speed=%.2f, distance=%.2f)",
                angle, speed, distance));
        mDrivetrain.translateDistance(angle, speed, distance);
        waitForTarget(new AnyMotorReachedTarget());
    }

    public void rotate(RobotDrivetrain.RotationDirection direction, double speed) {
        info(String.format(Locale.US, "rotate(direction=%s, speed=%.2f)", direction, speed));
        mDrivetrain.rotate(direction, speed);
    }

    private class GyroReachedHeading implements DistanceFromTarget {
        private RobotDrivetrain.RotationDirection mRotationDirection;
        private int mInitialHeading, mDesiredHeading, mLastHeading;
        private int mCurrentHeadingAdjustment = 0;
        private int mDesiredHeadingAdjustment = 0;
        private int mTotalDegrees;

        public GyroReachedHeading(RobotDrivetrain.RotationDirection rotationDirection,
                                  int initialHeading, int desiredHeading) {
            mRotationDirection = rotationDirection;
            mInitialHeading = mLastHeading = initialHeading;
            mDesiredHeading = desiredHeading;

            if (mRotationDirection == RobotDrivetrain.RotationDirection.LEFT &&
                    mInitialHeading < mDesiredHeading) {
                // Allow for zero-crossing anticlockwise 0 -> 359.
                mCurrentHeadingAdjustment += 360;
            }

            if (mRotationDirection == RobotDrivetrain.RotationDirection.RIGHT &&
                    mInitialHeading > mDesiredHeading) {
                // Allow for zero-crossing clockwise 359 -> 0.
                mDesiredHeadingAdjustment += 360;
            }

            mTotalDegrees = Math.abs((mInitialHeading+mCurrentHeadingAdjustment) -
                    (mDesiredHeading+mDesiredHeadingAdjustment));
        }

        @Override
        public double distance() {
            int currentHeading = mGyroSensor.getHeading();

            if (mRotationDirection == RobotDrivetrain.RotationDirection.LEFT) {
                if (currentHeading > mLastHeading && currentHeading > 350)
                    mCurrentHeadingAdjustment -= 360;
            } else {
                if (currentHeading < mLastHeading && currentHeading < 10)
                    mDesiredHeadingAdjustment -= 360;
            }

            int distanceDegrees = (currentHeading+mCurrentHeadingAdjustment) -
                    (mDesiredHeading+mDesiredHeadingAdjustment);

            distanceDegrees *=
                    (mRotationDirection == RobotDrivetrain.RotationDirection.LEFT) ? 1 : -1;

            double distance;
            if (distanceDegrees > 10)
                distance = 1.0;
            else if (distanceDegrees > 0)
                distance = (double) distanceDegrees / 10;
            else
                distance = 0.0;

            mOpMode.telemetry.addData("0. total", mTotalDegrees);
            mOpMode.telemetry.addData("1. current", String.format(Locale.US, "%d + %d",
                    currentHeading, mCurrentHeadingAdjustment));
            mOpMode.telemetry.addData("2. desired", String.format(Locale.US, "%d + %d",
                    mDesiredHeading, mDesiredHeadingAdjustment));
            mOpMode.telemetry.addData("3. distance", distance);
            mOpMode.telemetry.addData("4. degrees", distanceDegrees);
            mOpMode.telemetry.update();

            mLastHeading = currentHeading;
            return distance;
        }
    }
    public void rotateToHeading(RobotDrivetrain.RotationDirection direction,
                                double speed, int heading) {
        rotate(direction, speed);
        waitForTarget(new GyroReachedHeading(direction, mGyroSensor.getHeading(), heading));
    }
}
