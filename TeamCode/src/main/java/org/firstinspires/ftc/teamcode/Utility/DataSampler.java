package org.firstinspires.ftc.teamcode.Utility;

public class DataSampler {

    public enum SamplingMethod {
        AVERAGE,
        WEIGHTED_RECENCY,
        EXPONENTIAL_RECENCY
    }

    private SamplingMethod method;
    private double[] dataList;

    private int exponentialBase;


    // constructors

    public DataSampler(SamplingMethod method, int sampleSize, int exponentialBase) {

        this.method = method;
        this.exponentialBase = exponentialBase;
        dataList = new double[sampleSize];

    }

    public DataSampler(SamplingMethod method, int sampleSize) {
        this(method, sampleSize, 2);
    }

    public DataSampler(int sampleSize) {
        this(SamplingMethod.AVERAGE, sampleSize);
    }

    // methods

    public void updateData(double data) {
        for (int i = 0; i < dataList.length - 1; i++) {
            dataList[i] = dataList[i + 1];
        }
        dataList[dataList.length - 1] = data;
    }

    public double calculateData() {

        switch (this.method) {
            case AVERAGE:
                return getAverage();

            case WEIGHTED_RECENCY:
                return getWeightedRecency();

            case EXPONENTIAL_RECENCY:
                return getExponentialRecency(exponentialBase);

            default:
                return 0;
        }

    }

    // calculation methods

        // average
    private double getAverage() {
        double average = 0;
        for (int i = 0; i < dataList.length; i++) {
            average += dataList[i];
        }
        average /= dataList.length;
        return average;
    }


        // weighted recency
    private double getWeightedRecency() {
        double weightedAverage = 0;
        for (int i = 0; i < dataList.length; i++) {
            // Multiply each data sample by its position (index + 1) to give more weight to recent samples
            weightedAverage += dataList[i] * (i + 1);
        }
        // Normalize the weighted sum by dividing by the sum of the weights
        weightedAverage /= (dataList.length * (dataList.length + 1)) / 2;
        return weightedAverage;
    }

        // exponential recency
    private double getExponentialRecency(double base) {
        double weightedAverage = 0;
        double totalWeight = 0;
        for (int i = 0; i < dataList.length; i++) {
            double weight = Math.pow(base, i); // Base will control the exponential curve; larger = steeper, smaller = more gradual
            weightedAverage += dataList[i] * weight;
            totalWeight += weight;
        }
        weightedAverage /= totalWeight;
        return weightedAverage;
    }

}
