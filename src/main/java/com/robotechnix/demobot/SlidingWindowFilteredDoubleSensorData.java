package com.robotechnix.demobot;

public class SlidingWindowFilteredDoubleSensorData implements SensorDataProvider<Double> {
    SensorDataProvider<Double> mSensorDataProvider;
    int mWindowSize;
    double[] mData;
    int mIndex = 0;
    Double mValueCache;

    SlidingWindowFilteredDoubleSensorData(int windowSize, SensorDataProvider<Double> sensorDataProvider) {
        mSensorDataProvider = sensorDataProvider;
        mWindowSize = windowSize;
        mData = new double[mWindowSize];

        // Initially populate the array with the current value.
        populate(mSensorDataProvider.value());
    }

    synchronized private void populate(double value) {
        for (int i=0; i < mData.length; i++)
            mData[i] = value;
    }

    synchronized private void integrate(double value) {
        mData[mIndex] = value;
        mIndex = (mIndex+1) % mData.length;
        mValueCache = null;
    }

    synchronized private double average() {
        double sum = 0.0;
        for (double value : mData) {
            sum += value;
        }
        return sum / mData.length;
    }

    @Override
    public Double value() {
        integrate(mSensorDataProvider.value());
        return mValueCache = average();
    }
}
