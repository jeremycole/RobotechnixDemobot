package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public class RobotDrivetrain {
    public static class DriveParameters {
        private double mPower;
        private double mSlippage;

        public DriveParameters(double power, double slippage) {
            mPower = power;
            mSlippage = slippage;
        }

        public double getPower() {
            return mPower;
        }

        public void setPower(double power) {
            mPower = power;
        }

        public double getSlippage() {
            return mSlippage;
        }

        public void setSlippage(double slippage) {
            mSlippage = slippage;
        }
    }

    public static class DriveParametersFactory {
        private double mPower;
        private double mSlippage;

        public DriveParametersFactory(double power, double slippage) {
            mPower = power;
            mSlippage = slippage;
        }

        public DriveParameters power(double power) {
            return new DriveParameters(power, mSlippage);
        }

        public DriveParameters slippage(double slippage) {
            return new DriveParameters(mPower, slippage);
        }
    }

    public enum TranslationDirection {
        FORWARD         (0),
        STRAFE_RIGHT    (90),
        BACKWARD        (180),
        STRAFE_LEFT     (270);

        private int mAngle;

        TranslationDirection(int angle) {
            mAngle = angle;
        }

        public int getAngle() {
            return mAngle;
        }
    }

    public static class Translation {
        private int mAngle;
        private Map<RobotMotor, DriveParameters> mDrive;

        Translation(int angle, Map<RobotMotor, DriveParameters> drive) {
            mAngle = angle;
            mDrive = drive;
        }

        public int getAngle() {
            return mAngle;
        }

        public Map<RobotMotor, DriveParameters> getDrive() {
            return mDrive;
        }
    }

    public enum RotationDirection {
        LEFT    (+1),
        RIGHT   (-1);

        private int mRotation;
        RotationDirection(int rotation) {
            mRotation = rotation;
        }
    }

    public static class Rotation {
        private RotationDirection mRotationDirection;
        private Map<RobotMotor, DriveParameters> mDrive;

        Rotation(RotationDirection rotationDirection, Map<RobotMotor, DriveParameters> drive) {
            mRotationDirection = rotationDirection;
            mDrive = drive;
        }

        public RotationDirection getRotationDirection() {
            return mRotationDirection;
        }

        public Map<RobotMotor, DriveParameters> getDrive() {
            return mDrive;
        }
    }

    private Map<String, RobotMotor> mRobotMotors;
    private List<Translation> mTranslations;
    private List<Rotation> mRotations;
    private int mMaxSpeed = 1;
    private double mGearRatio = 1.0;
    private double mEncoderCountsPerRevolution = 1.0;
    private double mWheelDiameter = 1.0;

    private Map<RobotMotor, DriveParameters> mDrive;
    private Double mSpeed;
    private Integer mTranslationAngle;
    private RotationDirection mRotationDirection;

    RobotDrivetrain() {
        mRobotMotors = new HashMap<>();
        mTranslations = new ArrayList<>();
        mRotations = new ArrayList<>();
    }

    public int getMaxSpeed() {
        return mMaxSpeed;
    }

    public void setMaxSpeed(int maxSpeed) {
        mMaxSpeed = maxSpeed;
    }

    public double getGearRatio() {
        return mGearRatio;
    }

    public void setGearRatio(double gearRatio) {
        mGearRatio = gearRatio;
    }

    public double getEncoderCountsPerRevolution() {
        return mEncoderCountsPerRevolution;
    }

    public void setEncoderCountsPerRevolution(double encoderCountsPerRevolution) {
        mEncoderCountsPerRevolution = encoderCountsPerRevolution;
    }

    public double getWheelDiameter() {
        return mWheelDiameter;
    }

    public void setWheelDiameter(double wheelDiameter) {
        mWheelDiameter = wheelDiameter;
    }

    public double getWheelCircumference() {
        return mWheelDiameter * Math.PI;
    }

    public void addRobotMotor(RobotMotor motor) {
        motor.setDrivetrain(this);
        motor.getDcMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.getDcMotor().setMaxSpeed(mMaxSpeed);
        motor.getDcMotor().setPower(0.0);

        mRobotMotors.put(motor.getName(), motor);
    }

    public RobotMotor getRobotMotor(String name) {
        return mRobotMotors.get(name);
    }

    public void addTranslation(Translation translation) {
        mTranslations.add(translation);
    }

    public void addRotation(Rotation rotation) {
        mRotations.add(rotation);
    }

    private Map<RobotMotor, DriveParameters> blendDrives(Map<RobotMotor, DriveParameters> aDrive,
                                                double aWeight,
                                                Map<RobotMotor, DriveParameters> bDrive,
                                                double bWeight) {
        Map<RobotMotor, DriveParameters> drive = new HashMap<>();
        for (RobotMotor motor : aDrive.keySet()) {
            double aPower = aDrive.get(motor).getPower() * aWeight;
            double bPower = bDrive.get(motor).getPower() * bWeight;
            double aSlippage = aDrive.get(motor).getSlippage() * aWeight;
            double bSlippage = bDrive.get(motor).getSlippage() * bWeight;
            drive.put(motor, new DriveParameters(aPower + bPower, aSlippage + bSlippage));
        }

        return drive;
    }

    public Map<RobotMotor, DriveParameters> getTranslationDrive(int angle) {
        int l=-1, h=-1;
        for (int i = 0; i < mTranslations.size(); i++) {
            if (angle >= mTranslations.get(i).getAngle()) {
                l = i;
            }
        }

        if (l == -1)
            throw new RuntimeException("Couldn't find translation for angle");

        h = (l+1) % mTranslations.size();

        Translation lTranslation = mTranslations.get(l);
        Translation hTranslation = mTranslations.get(h);

        double lh1Distance = Math.abs(hTranslation.getAngle() - lTranslation.getAngle());
        double lh2Distance = Math.abs((hTranslation.getAngle()+360) - lTranslation.getAngle());
        double lhDistance = Math.min(lh1Distance, lh2Distance);
        double lWeight = 1.0 - (angle - lTranslation.getAngle()) / lhDistance;
        double hWeight = 1.0 - lWeight;

        Log.i("RobotDrivetrain", String.format(Locale.US,
                "getTranslationDrive: angle=%d, l=%d, h=%d, lWeight=%.2f, hWeight=%.2f, lhDistance=%.2f",
                angle, l, h, lWeight, hWeight, lhDistance));

        Map<RobotMotor, DriveParameters> drive = blendDrives(
                lTranslation.getDrive(), lWeight,
                hTranslation.getDrive(), hWeight);

        return drive;
    }

    public Map<RobotMotor, DriveParameters> getRotationDrive(RotationDirection direction) {
        for (Rotation rotation : mRotations) {
            if (rotation.getRotationDirection() == direction)
                return rotation.getDrive();
        }
        throw new RuntimeException("Couldn't find rotation drive for " + direction);
    }

    public double convertEncoderCountsToDistance(int counts) {
        return (counts / getEncoderCountsPerRevolution()) * getWheelCircumference();
    }

    public int convertDistanceToEncoderCounts(double distance) {
        return (int) ((distance / getWheelCircumference()) * getEncoderCountsPerRevolution());
    }

    public double getDistanceToPosition(int current, int target) {
        return convertEncoderCountsToDistance(Math.abs(target - current));
    }

    public double getDistanceToPosition(RobotMotor motor) {
        return getDistanceToPosition(
                motor.getDcMotor().getCurrentPosition(),
                motor.getDcMotor().getTargetPosition());
    }

    public void stop() {
        for (RobotMotor motor : mRobotMotors.values()) {
            motor.stop();
        }
    }

    private void setRobotMotorsDistance(Map<RobotMotor, DriveParameters> drive, double distance) {
        for (RobotMotor motor : drive.keySet()) {
            motor.setTargetPositionOffset(
                    convertDistanceToEncoderCounts(
                            Math.copySign(drive.get(motor).getSlippage() * distance,
                                    drive.get(motor).getPower())));
        }
    }

    private void setRobotMotorsPower(Map<RobotMotor, DriveParameters> drive, double speed) {
        for (RobotMotor motor : drive.keySet()) {
            motor.setPower(drive.get(motor).getPower() * speed);
        }
    }

    public boolean isAnyEncoderSatisfied() {
        for (RobotMotor motor : mRobotMotors.values()) {
            if (motor.isEncoderSatisfied())
                return true;
        }
        return false;
    }

    public double minDistanceUntilAnyEncoderSatisfied() {
        double min = 1.0;
        for (RobotMotor motor : mRobotMotors.values()) {
            if (motor.isEncoderSatisfied())
                min = Math.min(min, motor.distanceUntilEncoderSatisfied());
        }
        return min;
    }

    public void adjustSpeed(double factor) {
        if (factor != 1.0)
            setRobotMotorsPower(mDrive, Math.max(mSpeed * factor, 0.1));
    }

    public void translate(int angle, double speed) {
        mSpeed = speed;
        mTranslationAngle = angle;
        mRotationDirection = null;
        mDrive = getTranslationDrive(angle);
        setRobotMotorsPower(mDrive, speed);
    }

    public void translateDistance(int angle, double speed, double distance) {
        mSpeed = speed;
        mTranslationAngle = angle;
        mRotationDirection = null;
        mDrive = getTranslationDrive(angle);
        setRobotMotorsDistance(mDrive, distance);
        setRobotMotorsPower(mDrive, speed);
    }

    public void rotate(RotationDirection direction, double speed) {
        mSpeed = speed;
        mTranslationAngle = null;
        mRotationDirection = direction;
        mDrive = getRotationDrive(direction);
        setRobotMotorsPower(mDrive, speed);
    }
}
