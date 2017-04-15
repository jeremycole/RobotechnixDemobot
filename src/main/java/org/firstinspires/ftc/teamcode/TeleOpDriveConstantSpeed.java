/*
 * Copyright 2017, FTC Team 11574.
 *
 * A TeleOp program to allow control of the robot via a USB Gamepad.
 *
 * This program was initially based on code that was:
 *   Copyright (c) 2014, 2015 Qualcomm Technologies Inc
 */

package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

import java.util.Locale;

@TeleOp(name = "TeleOpDriveConstantSpeed", group = "Mecanum")
@SuppressWarnings({"unused", "WeakerAccess"})
public class TeleOpDriveConstantSpeed extends OpMode {
    DcMotor mFL, mBL, mFR, mBR;

    @Override
    public void init() {
        gamepad1.setJoystickDeadzone(0.05f);

        mFL = hardwareMap.dcMotor.get("mFL");
        mBL = hardwareMap.dcMotor.get("mBL");
        mFR = hardwareMap.dcMotor.get("mFR");
        mBR = hardwareMap.dcMotor.get("mBR");

        mFL.setDirection(DcMotor.Direction.REVERSE);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBR.setDirection(DcMotor.Direction.FORWARD);

        mFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mFL.setMaxSpeed(2500);
        mBL.setMaxSpeed(2500);
        mFR.setMaxSpeed(2500);
        mBR.setMaxSpeed(2500);
    }

    @Override
    public void loop() {
        double motorPower = 1.0;
        double x1 = gamepad1.left_stick_x;
        double y1 = gamepad1.left_stick_y;
        double x2 = gamepad1.right_stick_x;

        // Allow use of the analog left/right sticks to control the robot in differential
        // steering (tank driving) mode. Allow use of the left and right trigger buttons
        // to rotate the robot.
        mFL.setPower((y1 - x1 - x2) * motorPower);
        mBL.setPower((y1 + x1 - x2) * motorPower);
        mFR.setPower((y1 + x1 + x2) * motorPower);
        mBR.setPower((y1 - x1 + x2) * motorPower);
    }
}