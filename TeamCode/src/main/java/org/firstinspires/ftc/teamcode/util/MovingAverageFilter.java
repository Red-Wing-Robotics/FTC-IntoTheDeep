package org.firstinspires.ftc.teamcode.util;

/**
 * This filter is designed to smooth single-dimension sensor data. This should minimize
 * the outliers affecting the driving from a sensor read (such as a distance sensor).
 */
public class MovingAverageFilter {

    private final double[] readings;
    private int head;
    private int count;

    public MovingAverageFilter(int windowSize) {
        this.readings = new double[windowSize];
    }

    public void add(double value) {
        readings[head] = value;
        head = (head + 1) % readings.length;
        if (count < readings.length) {
            count++;
        }
    }

    public double getMovingAverage() {
        if (count == 0) {
            return Double.NaN;
        }
        double sum = 0;
        for (int i = 0; i < count; i++) {
            sum += readings[i];
        }
        return sum / count;
    }

}
