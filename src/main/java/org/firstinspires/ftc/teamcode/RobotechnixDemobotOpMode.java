package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RobotechnixDemobotOpMode extends LinearOpMode {
    RobotechnixDemobot robot;
    final private static String LOG_TAG = "RobotechnixDemobot";
    private void info(String msg) {
        Log.i(LOG_TAG, msg);
    }

    // If stop was requested, throw a StopImmediatelyException which will be
    // caught by runOpMode to stop the robot immediately.
    boolean shouldKeepRunning() {
        if(isStarted() && isStopRequested())
            throw new StopImmediatelyException();
        return true;
    }

    private void robotInitialize() {
        gamepad1.setJoystickDeadzone(0.05f);

        robot.initialize();
    }

    private void robotWaitForStart() {
        while (!isStarted() && !isStopRequested())
            idle();
    }

    public void robotRun() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotechnixDemobot(this);

        try {
            info("Calling robotInitialize()...");
            robotInitialize();

            info("Calling robotWaitForStart()...");
            robotWaitForStart();

            // Exit immediately if stop was pressed, otherwise continue.
            if (!isStarted() || isStopRequested()) {
                info("Stop requested!");
                return;
            }

            info("Calling robotRun()...");
            robotRun();

            // Stop just in case robotRun did not.
            robot.stop();
        } catch (Throwable t) {
            // Expected due to timer expiration or "Stop" button pressed.
            if (t instanceof StopImmediatelyException) {
                info("Stop requested!");
                robot.stop();
                return;
            }

            // Unexpected exception; log it, and then re-throw a RuntimeException.
            Log.e(LOG_TAG, "Exception caught!", t);

            if (t instanceof RuntimeException) {
                throw (RuntimeException) t;
            }

            throw new RuntimeException(t);
        }
    }

}
