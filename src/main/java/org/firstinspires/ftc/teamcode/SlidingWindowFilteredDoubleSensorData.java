package org.firstinspires.ftc.teamcode;

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
    }

    void populate(double value) {
        for (int i=0; i < mData.length; i++)
            mData[i] = value;
    }

    void integrate(double value) {
        mData[mIndex] = value;
        mIndex = (mIndex+1) % mData.length;
        mValueCache = null;
    }

    double average() {
        double sum = 0.0;
        for (double value : mData) {
            sum += value;
        }
        return sum / mData.length;
    }

    @Override
    public Double values() {
        integrate(mSensorDataProvider.values());
        return mValueCache = average();
    }
}
