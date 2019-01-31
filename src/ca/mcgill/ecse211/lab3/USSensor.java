package ca.mcgill.ecse211.lab3;

import java.util.Arrays;

import lejos.robotics.SampleProvider;
import lejos.utility.TimerListener;

public class USSensor implements TimerListener{
	
	
	private static final int BLOCK_THRESHOLD = 15;
	private int[] pastData;
	private volatile int median;
	private SampleProvider USPoller;
	private float[] USData;
	private Navigation navigator;
	
	public USSensor(SampleProvider USSampleProvider, float[] USSample, Navigation navigator) {
		// moving median filter of length 5
		this.pastData = new int[] {255,255,255,255,255};
		this.median = 255;
		this.USPoller = USSampleProvider;
		this.USData = USSample;
		this.navigator = navigator;
	}

	@Override
	public void timedOut() {
		USPoller.fetchSample(USData, 0);
//		System.out.println(USData[0]*100);
		// shift the past data to the left in the array
		for (int i = 0; i < pastData.length - 2; i++) {
			pastData[i] = pastData[i+1];
		}
		// add the sample to end of pastData
		pastData[pastData.length - 1] = (int) (USData[0] * 100);
		// calculate the median
		median = calculateMedian(pastData.clone());
		
		if (navigator.isNavigating() && median < BLOCK_THRESHOLD) {
			navigator.avoidObstacle();
		}
	}
	
	public int getFilteredDistance() {
		return median;
	}
	
	private static int calculateMedian(int[] data) {
		Arrays.sort(data);
		return data[data.length / 2];
	}
}