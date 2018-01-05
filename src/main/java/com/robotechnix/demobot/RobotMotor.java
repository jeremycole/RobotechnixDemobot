package com.robotechnix.demobot;

import com.qualcomm.robotcore.hardware.DcMotor;

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

    int getCurrentPosition() {
        return mDcMotor.getCurrentPosition();
    }

    void stop() {
        setPower(0.0);
    }
}
