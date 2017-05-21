package org.firstinspires.ftc.teamcode;

import android.util.ArrayMap;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

@SuppressWarnings("unused")
class RobotDrivetrain {
    static class EncoderCounters {
        private int current;
        private int target;
        private int offset;

        EncoderCounters(int encoderCounter) {
            current = encoderCounter;
            target = offset = 0;
        }

        public int getCurrent() {
            return current;
        }

        public void setCurrent(int current) {
            this.current = current;
        }

        public int getTarget() {
            return target;
        }

        public void setTarget(int target) {
            this.target = target;
        }

        public int getOffset() {
            return offset;
        }

        public void setOffset(int offset) {
            this.offset = offset;
        }

        public boolean isEncoderActive() {
            return (offset != 0);
        }

        public boolean isEncoderSatisfied() {
            if (offset > 0 && current >= target)
                return true;

            if (offset < 0 && current <= target)
                return true;

            return false;
        }

        public int getDistance() {
            return Math.abs(target - current);
        }
    }

    static class EncoderStatus {
        private Map<String, RobotMotor> mRobotMotors;
        private Map<RobotMotor, EncoderCounters> mEncoderStatus;

        EncoderStatus() {
            mEncoderStatus = new HashMap<>();
        }

        EncoderStatus(Map<String, RobotMotor> robotMotors) {
            mRobotMotors = robotMotors;
            mEncoderStatus = new HashMap<>();
            copy(robotMotors);
        }

        public Map<String, RobotMotor> getRobotMotors() {
            return mRobotMotors;
        }

        public void setRobotMotors(Map<String, RobotMotor> robotMotors) {
            mRobotMotors = robotMotors;
        }

        public Map<RobotMotor, EncoderCounters> getEncoderStatus() {
            return mEncoderStatus;
        }

        public void setEncoderStatus(Map<RobotMotor, EncoderCounters> encoderStatus) {
            mEncoderStatus = encoderStatus;
        }

        public void copy(Map<String, RobotMotor> motors) {
            for (RobotMotor motor : motors.values()) {
                mEncoderStatus.put(motor,
                        new EncoderCounters(motor.getDcMotor().getCurrentPosition()));
            }
        }

        public void update() {
            for (RobotMotor motor : mRobotMotors.values()) {
                if (mEncoderStatus.containsKey(motor))
                    mEncoderStatus.get(motor).setCurrent(motor.getCurrentPosition());
                else
                    mEncoderStatus.put(motor, new EncoderCounters(motor.getCurrentPosition()));
            }
        }

        public void setTargetOffset(RobotMotor motor, int offset) {
            EncoderCounters counters = mEncoderStatus.get(motor);
            counters.setOffset(offset);
            counters.setTarget(counters.current + offset);
        }

        public void clearTarget(RobotMotor motor) {
            EncoderCounters counters = mEncoderStatus.get(motor);
            counters.setOffset(0);
            counters.setTarget(0);
        }

        public void clearTarget() {
            for (RobotMotor motor : mRobotMotors.values()) {
                clearTarget(motor);
            }
        }

        public boolean isEncoderSatisfied(RobotMotor motor) {
            return mEncoderStatus.get(motor).isEncoderSatisfied();
        }

        public boolean isAnyEncoderSatisfied() {
            for (EncoderCounters counters : mEncoderStatus.values()) {
                if (!counters.isEncoderActive())
                    continue;

                if (counters.isEncoderSatisfied())
                    return true;
            }
            return false;
        }

        double minDistanceUntilAnyEncoderSatisfied() {
            double min = Double.MAX_VALUE;

            for (EncoderCounters counters : mEncoderStatus.values()) {
                if (!counters.isEncoderActive())
                    continue;

                min = Math.min(min, Math.abs(counters.getDistance()));
            }

            return min;
        }
    }

    static class DriveParameters {
        private double mPower;
        private double mSlippage;

        DriveParameters(double power, double slippage) {
            mPower = power;
            mSlippage = slippage;
        }

        double getPower() {
            return mPower;
        }

        void setPower(double power) {
            mPower = power;
        }

        double getSlippage() {
            return mSlippage;
        }

        void setSlippage(double slippage) {
            mSlippage = slippage;
        }
    }

    static class DriveParametersFactory {
        private double mPower;
        private double mSlippage;

        DriveParametersFactory(double power, double slippage) {
            mPower = power;
            mSlippage = slippage;
        }

        DriveParameters power(double power) {
            return new DriveParameters(power, mSlippage);
        }

        DriveParameters slippage(double slippage) {
            return new DriveParameters(mPower, slippage);
        }
    }

    enum TranslationDirection {
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

    static class Translation {
        private int mAngle;
        private Map<RobotMotor, DriveParameters> mDrive;

        Translation(int angle, Map<RobotMotor, DriveParameters> drive) {
            mAngle = angle;
            mDrive = drive;
        }

        int getAngle() {
            return mAngle;
        }

        Map<RobotMotor, DriveParameters> getDrive() {
            return mDrive;
        }
    }

    enum RotationDirection {
        LEFT    (+1),
        RIGHT   (-1);

        private int mRotation;
        RotationDirection(int rotation) {
            mRotation = rotation;
        }
    }

    static class Rotation {
        private RotationDirection mRotationDirection;
        private Map<RobotMotor, DriveParameters> mDrive;

        Rotation(RotationDirection rotationDirection, Map<RobotMotor, DriveParameters> drive) {
            mRotationDirection = rotationDirection;
            mDrive = drive;
        }

        RotationDirection getRotationDirection() {
            return mRotationDirection;
        }

        Map<RobotMotor, DriveParameters> getDrive() {
            return mDrive;
        }
    }

    Timer mDrivetrainTimer;
    TimerTask mDrivetrainTimerTask;

    private Map<String, RobotMotor> mRobotMotors;
    private List<Translation> mTranslations;
    private List<Rotation> mRotations;
    private EncoderStatus mEncoderStatus;
    private int mMaxSpeed = 1;
    private double mGearRatio = 1.0;
    private double mEncoderCountsPerRevolution = 1.0;
    private double mWheelDiameter = 1.0;

    private Map<RobotMotor, DriveParameters> mTranslationDrive;
    private Map<RobotMotor, DriveParameters> mRotationDrive;
    private Map<RobotMotor, DriveParameters> mBlendedDrive;
    private Double mTranslationSpeed;
    private Double mRotationSpeed;
    private Integer mTranslationAngle;
    private RotationDirection mRotationDirection;
    private Double mTargetSpeedAdjustment;

    RobotDrivetrain() {
        mRobotMotors = new HashMap<>();
        mTranslations = new ArrayList<>();
        mRotations = new ArrayList<>();
        mEncoderStatus = new EncoderStatus(mRobotMotors);
        mTargetSpeedAdjustment = null;
    }

    void run() {
        mDrivetrainTimer = new Timer();
        mDrivetrainTimerTask = new TimerTask() {
            @Override
            public void run() {
                tick();
            }
        };
        mDrivetrainTimer.schedule(mDrivetrainTimerTask, 0, 1);
    }

    void shutdown() {
        stopAllMotors();

        if (mDrivetrainTimer != null) {
            mDrivetrainTimer.cancel();
            mDrivetrainTimer = null;
        }
    }

    int getMaxSpeed() {
        return mMaxSpeed;
    }

    void setMaxSpeed(int maxSpeed) {
        mMaxSpeed = maxSpeed;
    }

    double getGearRatio() {
        return mGearRatio;
    }

    void setGearRatio(double gearRatio) {
        mGearRatio = gearRatio;
    }

    double getEncoderCountsPerRevolution() {
        return mEncoderCountsPerRevolution;
    }

    void setEncoderCountsPerRevolution(double encoderCountsPerRevolution) {
        mEncoderCountsPerRevolution = encoderCountsPerRevolution;
    }

    double getWheelDiameter() {
        return mWheelDiameter;
    }

    void setWheelDiameter(double wheelDiameter) {
        mWheelDiameter = wheelDiameter;
    }

    double getWheelCircumference() {
        return mWheelDiameter * Math.PI;
    }

    void addRobotMotor(RobotMotor motor) {
        motor.setDrivetrain(this);
        motor.getDcMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.getDcMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.getDcMotor().setPower(0.0);

        mRobotMotors.put(motor.getName(), motor);
    }

    RobotMotor getRobotMotor(String name) {
        return mRobotMotors.get(name);
    }

    void addTranslation(Translation translation) {
        mTranslations.add(translation);
    }

    void addRotation(Rotation rotation) {
        mRotations.add(rotation);
    }

    private Map<RobotMotor, DriveParameters> normalizeDrive(
            Map<RobotMotor, DriveParameters> drive, double factor) {
        Map<RobotMotor, DriveParameters> normalizedDrive = new ArrayMap<>();

        for (RobotMotor motor : drive.keySet()) {
            DriveParameters dp = drive.get(motor);
            normalizedDrive.put(motor,
                    new DriveParameters(dp.getPower() * factor, dp.getSlippage()));
        }

        return normalizedDrive;
    }

    synchronized private Map<RobotMotor, DriveParameters> blendDrives(
            Map<RobotMotor, DriveParameters> aDrive,
            double aWeight,
            Map<RobotMotor, DriveParameters> bDrive,
            double bWeight) {

        double maxPower = 0.0;
        Map<RobotMotor, DriveParameters> drive = new HashMap<>();

        for (RobotMotor motor : aDrive.keySet()) {
            double aPower = aDrive.get(motor).getPower() * aWeight;
            double bPower = bDrive.get(motor).getPower() * bWeight;
            double tPower = aPower + bPower;

            double aSlippage = aDrive.get(motor).getSlippage() * aWeight;
            double bSlippage = bDrive.get(motor).getSlippage() * bWeight;
            double tSlippage = aSlippage + bSlippage;

            maxPower = Math.max(maxPower, Math.abs(tPower));
            drive.put(motor, new DriveParameters(tPower, tSlippage));
        }

        if (maxPower > 1.0)
            drive = normalizeDrive(drive, 1.0 / maxPower);

        return drive;
    }

    Map<RobotMotor, DriveParameters> makeTranslationDrive(int angle) {
        int l=-1, h;
        for (int i = 0; i < mTranslations.size(); i++) {
            if (angle >= mTranslations.get(i).getAngle()) {
                l = i;
            }
        }

        if (l == -1)
            throw new RuntimeException("Couldn't find translation for angle " + angle);

        h = (l+1) % mTranslations.size();

        Translation lTranslation = mTranslations.get(l);
        Translation hTranslation = mTranslations.get(h);

        double lh1Distance = Math.abs(hTranslation.getAngle() - lTranslation.getAngle());
        double lh2Distance = Math.abs((hTranslation.getAngle()+360) - lTranslation.getAngle());
        double lhDistance = Math.min(lh1Distance, lh2Distance);
        double lWeight = 1.0 - (angle - lTranslation.getAngle()) / lhDistance;
        double hWeight = 1.0 - lWeight;

        return blendDrives(lTranslation.getDrive(), lWeight, hTranslation.getDrive(), hWeight);
    }

    Map<RobotMotor, DriveParameters> makeRotationDrive(RotationDirection direction) {
        for (Rotation rotation : mRotations) {
            if (rotation.getRotationDirection() == direction)
                return rotation.getDrive();
        }
        throw new RuntimeException("Couldn't find rotation drive for " + direction);
    }

    private double convertEncoderCountsToDistance(int counts) {
        return (counts / getEncoderCountsPerRevolution()) * getWheelCircumference();
    }

    private int convertDistanceToEncoderCounts(double distance) {
        return (int) ((distance / getWheelCircumference()) * getEncoderCountsPerRevolution());
    }

    synchronized public Map<RobotMotor, DriveParameters> getTranslationDrive() {
        return mTranslationDrive;
    }

    synchronized public void setTranslationDrive(Map<RobotMotor, DriveParameters> translationDrive) {
        mTranslationDrive = translationDrive;
    }

    synchronized public Map<RobotMotor, DriveParameters> getRotationDrive() {
        return mRotationDrive;
    }

    synchronized public void setRotationDrive(Map<RobotMotor, DriveParameters> rotationDrive) {
        mRotationDrive = rotationDrive;
    }

    synchronized public Map<RobotMotor, DriveParameters> getBlendedDrive() {
        return mBlendedDrive;
    }

    synchronized public void setBlendedDrive(Map<RobotMotor, DriveParameters> blendedDrive) {
        mBlendedDrive = blendedDrive;
    }

    public Double getTargetSpeedAdjustment() {
        return mTargetSpeedAdjustment;
    }

    public void setTargetSpeedAdjustment(Double targetSpeedAdjustment) {
        mTargetSpeedAdjustment = targetSpeedAdjustment;
    }

    public void clearTargetSpeedAdjustment() {
        mTargetSpeedAdjustment = null;
    }

    boolean isStoppedAllMotors() {
        return getBlendedDrive() == null;
    }

    void stopTranslation() {
        setTranslationDrive(null);
    }

    void stopRotation() {
        setRotationDrive(null);
    }

    void stopAllMotors() {
        setTranslationDrive(null);
        setRotationDrive(null);
        setBlendedDrive(null);
        for (RobotMotor motor : mRobotMotors.values()) {
            motor.stop();
        }
    }

    synchronized private void setRobotMotorsPower(Map<RobotMotor, DriveParameters> drive) {
        if (drive == null) {
            stopAllMotors();
            return;
        }

        for (RobotMotor motor : drive.keySet()) {
            motor.setPower(drive.get(motor).getPower());
        }
    }

    synchronized private void setRobotMotorsDistance(
            Map<RobotMotor, DriveParameters> drive,
            double distance) {
        for (RobotMotor motor : drive.keySet()) {
            DriveParameters parameters = drive.get(motor);
            mEncoderStatus.setTargetOffset(motor, convertDistanceToEncoderCounts(
                    Math.copySign(parameters.getSlippage() * distance,
                                  parameters.getPower())));
        }
    }

    boolean isAnyEncoderSatisfied() {
        return mEncoderStatus.isAnyEncoderSatisfied();
    }

    double minDistanceUntilAnyEncoderSatisfied() {
        return Math.min(1.0, 2.0 * mEncoderStatus.minDistanceUntilAnyEncoderSatisfied() / getEncoderCountsPerRevolution());
    }

    private void adjustSpeed(double factor) {
        if (mBlendedDrive != null) {
            mBlendedDrive = normalizeDrive(mBlendedDrive, Math.max(0.5, factor));
            setRobotMotorsPower(mBlendedDrive);
        }
    }

    synchronized private Map<RobotMotor, DriveParameters> blendTranslationAndRotationDrives() {
        if (mTranslationDrive != null && mRotationDrive != null)
            return blendDrives(mTranslationDrive, mTranslationSpeed, mRotationDrive, mRotationSpeed);
        else if (mTranslationDrive != null)
            return normalizeDrive(mTranslationDrive, mTranslationSpeed);
        else if (mRotationDrive != null)
            return normalizeDrive(mRotationDrive, mRotationSpeed);

        return null;
    }

    void translate(int angle, double speed) {
        mTranslationSpeed = speed;
        mTranslationAngle = angle;
        if (speed > 0.0)
            setTranslationDrive(makeTranslationDrive(angle));
        else
            setTranslationDrive(null);
    }

    void translateDistance(int angle, double speed, double distance) {
        mTranslationSpeed = speed;
        mTranslationAngle = angle;
        setTranslationDrive(makeTranslationDrive(angle));
        setRobotMotorsDistance(getTranslationDrive(), distance);
    }

    void rotate(RotationDirection direction, double speed) {
        mRotationSpeed = speed;
        mRotationDirection = direction;
        if (speed > 0.0)
            setRotationDrive(makeRotationDrive(direction));
        else
            setRotationDrive(null);
    }

    void tick() {
        mEncoderStatus.update();
        setBlendedDrive(blendTranslationAndRotationDrives());
        if (mTargetSpeedAdjustment != null)
            adjustSpeed(mTargetSpeedAdjustment);
        setRobotMotorsPower(getBlendedDrive());
    }
}
