package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Locale;

public class RobotMotor {
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

    public RobotDrivetrain getDrivetrain() {
        return mDrivetrain;
    }

    public void setDrivetrain(RobotDrivetrain drivetrain) {
        mDrivetrain = drivetrain;
    }

    public String getName() {
        return mName;
    }

    public void setName(String name) {
        mName = name;
    }

    public DcMotor getDcMotor() {
        return mDcMotor;
    }

    public void setDcMotor(DcMotor dcMotor) {
        mDcMotor = dcMotor;
    }

    public void setPower(double power) {
        mDcMotor.setPower(power);
    }

    public void setTargetPositionOffset(int targetPositionOffset) {
        mDcMotor.setTargetPosition(mDcMotor.getCurrentPosition() + targetPositionOffset);
        Log.i("RobotMotor", String.format(Locale.US, "%s.setTargetPositionOffset(targetPositionOffset=%d) -> getCurrentPosition=%d, getTargetPosition=%d",
                this.getName(), targetPositionOffset, mDcMotor.getCurrentPosition(), mDcMotor.getTargetPosition()));
    }

    public boolean isEncoderSatisfied() {
        if (mDcMotor.getPower() == 0.0)
            return true;
        else if (mDcMotor.getPower() > 0.0)
            return mDcMotor.getCurrentPosition() >= mDcMotor.getTargetPosition();
        else
            return mDcMotor.getCurrentPosition() <= mDcMotor.getTargetPosition();
    }

    public double distanceUntilEncoderSatisfied() {
        double delta = Math.abs(mDcMotor.getCurrentPosition() - mDcMotor.getTargetPosition());

        if (mDcMotor.getPower() == 0.0 || isEncoderSatisfied())
            return 0.0;
        else
            return Math.min(1.0, delta / (mDcMotor.getPower() * 100.0));
    }

    public void stop() {
        setPower(0.0);
    }
}
