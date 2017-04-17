package org.firstinspires.ftc.teamcode;

public class Sensor<T> {
    private SensorDataProvider<T> mSensorDataProvider;
    private T mData;

    Sensor(SensorDataProvider<T> sensorDataProvider) {
        mSensorDataProvider = sensorDataProvider;
    }

    synchronized void collect() {
        mData = mSensorDataProvider.values();
    }

    synchronized T getData() {
        return mData;
    }
}
