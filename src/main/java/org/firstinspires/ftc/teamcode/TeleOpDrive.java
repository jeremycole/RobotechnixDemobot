/*
 * Copyright 2017, FTC Team 11574.
 *
 * A TeleOp program to allow control of the robot via a USB Gamepad.
 *
 * This program was initially based on code that was:
 *   Copyright (c) 2014, 2015 Qualcomm Technologies Inc
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpDrive", group = "TeleOp")
@SuppressWarnings({"unused", "WeakerAccess"})
public class TeleOpDrive extends RobotechnixDemobotOpMode {
    @Override
    public void robotRun() {
        double motorPower = 1.0;
        RobotMotor mFL, mBL, mFR, mBR;

        mFL = robot.mDrivetrain.getRobotMotor("mFL");
        mBL = robot.mDrivetrain.getRobotMotor("mBL");
        mFR = robot.mDrivetrain.getRobotMotor("mFR");
        mBR = robot.mDrivetrain.getRobotMotor("mBR");

        while (shouldKeepRunning()) {
            double x1 = -gamepad1.left_stick_x;
            double y1 = -gamepad1.left_stick_y;
            double x2 = -gamepad1.right_stick_x;
            double lt = -gamepad1.left_trigger;
            double rt = gamepad1.right_trigger;

            robot.positionRangeServo((lt + rt + 1.0) / 2.0);

            // Allow use of the analog left/right sticks to control the robot in differential
            // steering (tank driving) mode. Allow use of the left and right trigger buttons
            // to rotate the robot.
            mFL.setPower((y1 - x1 - x2) * motorPower);
            mBL.setPower((y1 + x1 - x2) * motorPower);
            mFR.setPower((y1 + x1 + x2) * motorPower);
            mBR.setPower((y1 - x1 + x2) * motorPower);

            telemetry.addData("1. gyro", robot.mGyroSensor.getHeading());
            telemetry.addData("2. compass", robot.mCompassSensor.getDirection());
            telemetry.addData("3. range", robot.mRangeSensor.getVoltage());
            telemetry.update();
        }
    }
}