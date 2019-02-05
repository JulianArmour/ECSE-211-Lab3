package ca.mcgill.ecse211.lab3;

import java.util.Arrays;

import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

/**
 * USSensor is the driver for the ultrasonic sensor. It periodically calls the
 * ultrasonic sensor to poll a sample using a timer. It then calculates
 * and stores the median distance for the past 5 samples. If this median is below
 * the {@link #BLOCK_THRESHOLD} then it calls {@link Navigation} to avoid an obstacle.
 * 
 * @author Julian Armour, Alice Kazarine
 * @version 1.0
 * @since 2019-02-01
 */
public class USSensor implements TimerListener {

    private static final int BLOCK_THRESHOLD = 15; // distance threshold for detecting an obstacle
    private int[] pastData;
    private int median;
    private SampleProvider usSampler;
    private float[] USData;
    private Navigation navigator;

    /**
     * 
     * @param USSampleProvider The ultrasonic sample provider
     * @param USSample         The data storage array for the sample
     */
    public USSensor(SampleProvider USSampleProvider, float[] USSample, Navigation navigator) {
        // moving median filter of length 5
        this.pastData = new int[] { 255, 255, 255, 255, 255 };
        this.median = 255;
        this.usSampler = USSampleProvider;
        this.USData = USSample;
        this.navigator = navigator;
    }

    /**
     * Polls data from the ultrasonic sensor, filter's this data using a median filter,
     * saves the median for access. Finally it checks if there is an obstacle close to the
     * robot. If so, it calls {@link Navigation#avoidObstacle()}.
     */
    @Override
    public void timedOut() {
        usSampler.fetchSample(USData, 0);
        // System.out.println(USData[0]*100);
        // shift the past data to the left in the array
        for (int i = 0; i < pastData.length - 1; i++) {
            pastData[i] = pastData[i + 1];
        }
        // add the sample to end of pastData
        pastData[pastData.length - 1] = (int) (USData[0] * 100);
        // calculate the median
        median = calculateMedian(pastData.clone());

        // System.out.println(navigator.getNavigationState());

        // only avoid obstacles when the robot is navigating
        if (navigator.isNavigating() && median < BLOCK_THRESHOLD) {
            // System.out.println("OBSTACLE");
            navigator.avoidObstacle();
        }
    }

    /**
     * 
     * @return The current filtered distance
     */
    public int getFilteredDistance() {
        return median;
        // return 255; // for testing
    }

    private static int calculateMedian(int[] data) {
        Arrays.sort(data);
        return data[data.length / 2];
    }
}
