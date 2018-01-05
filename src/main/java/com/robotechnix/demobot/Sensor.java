package com.robotechnix.demobot;

public class Sensor<T> {
    private SensorDataProvider<T> mSensorDataProvider;
    private T mData;

    Sensor(SensorDataProvider<T> sensorDataProvider) {
        mSensorDataProvider = sensorDataProvider;
    }

    synchronized void collect() {
        mData = mSensorDataProvider.value();
    }

    synchronized T value() {
        return mData;
    }
}
