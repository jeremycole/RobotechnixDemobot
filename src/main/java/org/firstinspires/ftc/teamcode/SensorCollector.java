package org.firstinspires.ftc.teamcode;

import java.util.LinkedList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

public class SensorCollector {
    interface OnSensorDataCollection<T> {
        void onSensorDataCollection(Sensor<T> sensor, long clock);
    }

    static class SensorItem {
        Sensor mSensor;
        int mFrequency;
        OnSensorDataCollection mOnSensorDataCollection;
        long mLastCollected;

        public SensorItem(int frequency, Sensor sensor,
                          OnSensorDataCollection onSensorDataCollection) {
            mFrequency = frequency;
            mSensor = sensor;
            mOnSensorDataCollection = onSensorDataCollection;
        }

        public long getLastCollected() {
            return mLastCollected;
        }

        public void setLastCollected(long lastCollected) {
            mLastCollected = lastCollected;
        }
    }

    int mFrequency;
    long mClock = 0;
    List<SensorItem> mSensors;

    Timer mSensorCollectorTimer;
    TimerTask mSensorCollectorTimerTask;

    SensorCollector(int frequency) {
        mFrequency = frequency;
        mSensors = new LinkedList<>();
    }

    synchronized void addSensor(int frequency, Sensor sensor, OnSensorDataCollection onSensorDataCollection) {
        mSensors.add(new SensorItem(frequency, sensor, onSensorDataCollection));
    }

    synchronized void tick() {
        mClock++;

        for (SensorItem sensorItem : mSensors) {
            if (mClock % sensorItem.mFrequency != 0)
                continue;

            sensorItem.mSensor.collect();
            sensorItem.setLastCollected(mClock);
            if (sensorItem.mOnSensorDataCollection != null) {
                sensorItem.mOnSensorDataCollection.
                        onSensorDataCollection(sensorItem.mSensor, mClock);
            }
        }
    }

    synchronized void run() {
        mSensorCollectorTimer = new Timer();
        mSensorCollectorTimerTask = new TimerTask() {
            @Override
            public void run() {
                tick();
            }
        };
        mSensorCollectorTimer.schedule(mSensorCollectorTimerTask, 0, mFrequency);
    }

    synchronized void shutdown() {
        if (mSensorCollectorTimer != null) {
            mSensorCollectorTimer.cancel();
            mSensorCollectorTimer = null;
        }
    }
}
