package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Locale;

@SuppressWarnings("unused")
class RobotechnixDemobot {
    private interface DistanceFromTarget {
        double distance();
    }

    private class AnyMotorReachedTarget implements DistanceFromTarget {
        @Override
        public double distance() {
            return mDrivetrain.minDistanceUntilAnyEncoderSatisfied();
        }
    }

    interface HeadingProvidingDevice {
        int heading();
    }

    private class GyroHeading implements HeadingProvidingDevice {
        @Override
        public int heading() {
            return mRawGyroSensor.getHeading();
        }
    }

    private class CompassHeading implements HeadingProvidingDevice {
        @Override
        public int heading() {
            return (int) mRawCompassSensor.getDirection();
        }
    }

    private class DeviceReachedHeading implements DistanceFromTarget {
        private HeadingProvidingDevice mDevice;
        private RobotDrivetrain.RotationDirection mRotationDirection;
        private int mInitialHeading, mDesiredHeading, mLastHeading;
        private int mCurrentHeadingAdjustment = 0;
        private int mDesiredHeadingAdjustment = 0;
        private int mTotalDegrees;

        DeviceReachedHeading(HeadingProvidingDevice device,
                                    RobotDrivetrain.RotationDirection rotationDirection,
                                    int initialHeading, int desiredHeading) {
            mDevice = device;
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
            int currentHeading = mDevice.heading();

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

    private class IrSeekerPointedAtBeacon implements DistanceFromTarget {
        double mInitialAngle;
        IrSeekerPointedAtBeacon() {
            mInitialAngle = mRawIrSeekerSensor.getAngle();
        }

        @Override
        public double distance() {
            double angle = mRawIrSeekerSensor.getAngle();
            mOpMode.telemetry.addData("angle", angle);
            mOpMode.telemetry.update();

            if (mInitialAngle < 0 && angle > 0)
                return 0.0;

            if (mInitialAngle > 0 && angle < 0)
                return 0.0;

            return Math.min(1.0, Math.abs(angle) / 30.0);
        }
    }

    private class IrSeekerBeaconInRange implements DistanceFromTarget {
        @Override
        public double distance() {
            return mRawIrSeekerSensor.getStrength() > 20 ? 0.0 : 1.0;
        }
    }

    final private static String LOG_TAG = "RobotechnixDemobot";
    private void info(String msg) {
        Log.i(LOG_TAG, msg);
    }

    private LinearOpMode mOpMode;

    RobotechnixDemobot(LinearOpMode opmode) {
        mOpMode = opmode;
    }

    // If stop was requested, throw a StopImmediatelyException which will be
    // caught by runOpMode to stop the robot immediately.
    private boolean shouldKeepRunning() {
        if(mOpMode.isStarted() && mOpMode.isStopRequested())
            throw new StopImmediatelyException();
        return true;
    }

    RobotDrivetrain mDrivetrain;

    final private static double DRIVE_SLIPPAGE_FACTOR  = 1.00;
    final private static double STRAFE_SLIPPAGE_FACTOR = 1.08;
    final private static double ROTATE_SLIPPAGE_FACTOR = 1.00;

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

        mDrivetrain.run();
    }

    private Servo mRangeServo;

    private void initializeServos() {
        mRangeServo = mOpMode.hardwareMap.servo.get("range_servo");
        mRangeServo.setDirection(Servo.Direction.FORWARD);
        mRangeServo.scaleRange(-1.0, +1.0);
        mRangeServo.setPosition(0.5);
    }

    SensorCollector mSensorCollector;

    GyroSensor mRawGyroSensor;
    Sensor<Double> mGyroSensor;

    ModernRoboticsI2cCompassSensor mRawCompassSensor;
    Sensor<Double> mCompassSensor;

    AnalogInput mRawRangeSensor;
    Sensor<Double> mRangeSensor;

    IrSeekerSensor mRawIrSeekerSensor;
    Sensor<Double> mIrSeekerAngleSensor;
    Sensor<Double> mIrSeekerStrengthSensor;

    private void initializeSensors() {
        mSensorCollector = new SensorCollector(1);

        mRawGyroSensor = mOpMode.hardwareMap.gyroSensor.get("gyro");
        mRawGyroSensor.calibrate();

        mGyroSensor = new Sensor<>(new SlidingWindowFilteredDoubleSensorData(10,
                new SensorDataProvider<Double>() {
            @Override
            public Double values() {
                return (double) mRawGyroSensor.getHeading();
            }
        }));
        mSensorCollector.addSensor(1, mGyroSensor, null);

        mOpMode.telemetry.addData(">", "Calibrating gyro...");
        mOpMode.telemetry.update();

        mRawCompassSensor = mOpMode.hardwareMap.get(
                ModernRoboticsI2cCompassSensor.class, "compass");
        mRawCompassSensor.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);

        mCompassSensor = new Sensor<Double>(new SlidingWindowFilteredDoubleSensorData(10,
                new SensorDataProvider<Double>() {
                    @Override
                    public Double values() {
                        return mRawCompassSensor.getDirection();
                    }
                }));

        mSensorCollector.addSensor(1, mCompassSensor, null);

        mRawRangeSensor = mOpMode.hardwareMap.analogInput.get("range");

        mRangeSensor = new Sensor<Double>(new SlidingWindowFilteredDoubleSensorData(10,
                new SensorDataProvider<Double>() {
                    @Override
                    public Double values() {
                        return mRawRangeSensor.getVoltage();
                    }
                }));
        mSensorCollector.addSensor(1, mRangeSensor, null);

        mRawIrSeekerSensor = mOpMode.hardwareMap.irSeekerSensor.get("ir");

        mIrSeekerAngleSensor = new Sensor<>(new SlidingWindowFilteredDoubleSensorData(50,
                new SensorDataProvider<Double>() {
                    @Override
                    public Double values() {
                        return mRawIrSeekerSensor.getAngle();
                    }
                }));

        mSensorCollector.addSensor(1, mIrSeekerAngleSensor, null);

        mIrSeekerStrengthSensor = new Sensor<>(new SlidingWindowFilteredDoubleSensorData(50,
                new SensorDataProvider<Double>() {
                    @Override
                    public Double values() {
                        return mRawIrSeekerSensor.getStrength();
                    }
                }));

        mSensorCollector.addSensor(1, mIrSeekerStrengthSensor, null);

        // Wait for gyro calibration to finish before returning.
        while (shouldKeepRunning() && mRawGyroSensor.isCalibrating())
            mOpMode.idle();

        mSensorCollector.run();
    }

    void initialize() {
        initializeDrivetrain();
        initializeServos();
        initializeSensors();

        mOpMode.telemetry.addData(">", "Ready!");
        mOpMode.telemetry.update();
    }

    void stop() {
        if (!mDrivetrain.isStoppedAllMotors()) {
            rotate(null, 0.0);
            translate(0, 0.0);
        }
    }

    private void waitForTarget(DistanceFromTarget distanceFromTarget) {
        double distance;
        do {
            distance = distanceFromTarget.distance();
            if (distance < 1.0)
                mDrivetrain.adjustSpeed(distance);
        } while (shouldKeepRunning() && distance > 0.0);
    }

    void translate(int angle, double speed) {
        //info(String.format(Locale.US, "translate(angle=%d, speed=%.2f)", angle, speed));
        mDrivetrain.translate(angle, speed);
    }

    void translateDistance(int angle, double speed, double distance) {
        //info(String.format(Locale.US, "translateDistance(angle=%d, speed=%.2f, distance=%.2f)",
         //       angle, speed, distance));
        mDrivetrain.translateDistance(angle, speed, distance);
        waitForTarget(new AnyMotorReachedTarget());
    }

    void rotate(RobotDrivetrain.RotationDirection direction, double speed) {
        //info(String.format(Locale.US, "rotate(direction=%s, speed=%.2f)", direction, speed));
        mDrivetrain.rotate(direction, speed);
    }

    void rotateToHeading(RobotDrivetrain.RotationDirection direction,
                                double speed, int heading) {
        HeadingProvidingDevice device = new CompassHeading();
        int startHeading = device.heading();
        rotate(direction, speed);
        waitForTarget(new DeviceReachedHeading(device, direction, startHeading, heading));
    }

    void rotateDegrees(RobotDrivetrain.RotationDirection direction,
                              double speed, int degrees) {
        HeadingProvidingDevice device = new CompassHeading();
        int startHeading = device.heading();
        int desiredHeading;

        if(direction == RobotDrivetrain.RotationDirection.LEFT)
            desiredHeading = startHeading - degrees;
        else
            desiredHeading = startHeading + degrees;

        desiredHeading %= 360;

        rotate(direction, speed);
        waitForTarget(new DeviceReachedHeading(device, direction, startHeading, desiredHeading));
    }

    void rotateToIrBeacon(double speed) {
        double angle = mRawIrSeekerSensor.getAngle();

        if (Math.abs(angle) < 2) {
            return;
        } else if (angle < 0) {
            rotate(RobotDrivetrain.RotationDirection.LEFT, speed);
        } else {
            rotate(RobotDrivetrain.RotationDirection.RIGHT, speed);
        }

        waitForTarget(new IrSeekerPointedAtBeacon());
    }

    void positionRangeServo(double position) {
        mRangeServo.setPosition(position-0.03);
    }
}
