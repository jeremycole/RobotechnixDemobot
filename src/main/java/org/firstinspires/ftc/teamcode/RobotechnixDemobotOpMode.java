package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RobotechnixDemobotOpMode extends LinearOpMode {
    RobotechnixDemobot robot;
    final private static String LOG_TAG = "RobotechnixDemobot";
    private void info(String msg) {
        Log.i(LOG_TAG, msg);
    }

    // If shutdown was requested, throw a StopImmediatelyException which will be
    // caught by runOpMode to shutdown the robot immediately.
    boolean shouldKeepRunning() {
        if(isStarted() && isStopRequested()) {
            robotStop();
            throw new StopImmediatelyException();
        }
        return true;
    }

    private void robotInitialize() {
        gamepad1.setJoystickDeadzone(0.01f);

        robot.initialize();
    }

    private void robotWaitForStart() {
        while (!isStarted() && !isStopRequested())
            idle();
    }

    public void robotRun() {
    }

    public void robotStop() {
        robot.shutdown();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotechnixDemobot(this);

        try {
            info("Calling robotInitialize()...");
            robotInitialize();

            info("Calling robotWaitForStart()...");
            robotWaitForStart();

            // Exit immediately if shutdown was pressed, otherwise continue.
            if (!isStarted() || isStopRequested()) {
                info("Stop requested!");
                return;
            }

            info("Calling robotRun()...");
            robotRun();

            info("Calling robotStop()...");
            robotStop();
        } catch (Throwable t) {
            // Expected due to timer expiration or "Stop" button pressed.
            if (t instanceof StopImmediatelyException) {
                info("Stop was requested!");
                return;
            }

            // Unexpected exception; log it, and then re-throw a RuntimeException.
            Log.e(LOG_TAG, "Exception caught!", t);

            if (t instanceof RuntimeException) {
                throw (RuntimeException) t;
            }

            throw new RuntimeException(t);
        } finally {
            // Stop the robot, as we're exiting either way.
            info("Calling robotStop() in finally block...");
            robotStop();
        }
    }

}
