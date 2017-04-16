package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Locale;

@SuppressWarnings("unused")
class RobotMotor {
    private RobotDrivetrain mDrivetrain;
    private String mName;
    private DcMotor mDcMotor;

    RobotMotor(String name,
               DcMotor dcMotor,
               DcMotor.Direction direction) {
        mName = name;
        mDcMotor = dcMotor;

        mDcMotor.setDirection(direction);
    }

    RobotDrivetrain getDrivetrain() {
        return mDrivetrain;
    }

    void setDrivetrain(RobotDrivetrain drivetrain) {
        mDrivetrain = drivetrain;
    }

    String getName() {
        return mName;
    }

    void setName(String name) {
        mName = name;
    }

    DcMotor getDcMotor() {
        return mDcMotor;
    }

    void setDcMotor(DcMotor dcMotor) {
        mDcMotor = dcMotor;
    }

    void setPower(double power) {
        mDcMotor.setPower(power);
    }

    void setTargetPositionOffset(int targetPositionOffset) {
        mDcMotor.setTargetPosition(mDcMotor.getCurrentPosition() + targetPositionOffset);
        Log.i("RobotMotor", String.format(Locale.US, "%s.setTargetPositionOffset(targetPositionOffset=%d) -> getCurrentPosition=%d, getTargetPosition=%d",
                this.getName(), targetPositionOffset, mDcMotor.getCurrentPosition(), mDcMotor.getTargetPosition()));
    }

    boolean isEncoderSatisfied() {
        if (mDcMotor.getPower() == 0.0)
            return true;
        else if (mDcMotor.getPower() > 0.0)
            return mDcMotor.getCurrentPosition() >= mDcMotor.getTargetPosition();
        else
            return mDcMotor.getCurrentPosition() <= mDcMotor.getTargetPosition();
    }

    double distanceUntilEncoderSatisfied() {
        double delta = Math.abs(mDcMotor.getCurrentPosition() - mDcMotor.getTargetPosition());

        if (mDcMotor.getPower() == 0.0 || isEncoderSatisfied())
            return 0.0;
        else
            return Math.min(1.0, delta / (mDcMotor.getPower() * 100.0));
    }

    void stop() {
        setPower(0.0);
    }
}
