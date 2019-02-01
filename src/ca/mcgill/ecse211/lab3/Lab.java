// Lab2.java
package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;

public class Lab {
	// the list of waypoints is defined as an array
	private static int[][] map;
	
	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final NXTRegulatedMotor sweepMotor = new NXTRegulatedMotor(LocalEV3.get().getPort("D"));

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double TILE_SIZE = 30.48;
	// The radius of the wheels in cm
	public static final double WHEEL_RAD = 2.1; // Radius increase = distance decrease
	// The distance between both wheels in cm
	public static final double TRACK = 11.0; // Width decrease = turn angle increase
	private static final int MOTOR_ACCELERATION = 1000;

	private static final int DISTANCE_POLL_PERIOD = 100;

	// defined port and sensor
	private static  Port portUS;
	private static SensorModes us;
	private static SampleProvider distanceProvider;
	private static float[] sampleUS;
	
	// Odometer for keeping track of position
	private static Odometer odometer;
	// ultrasonic sensor
	private static USSensor distanceSensor;

	private static Navigation navigator;

	public static void main(String[] args) throws OdometerExceptions {
		leftMotor.setAcceleration(MOTOR_ACCELERATION);
		rightMotor.setAcceleration(MOTOR_ACCELERATION);
		
		portUS = LocalEV3.get().getPort("S2");
		us = new EV3UltrasonicSensor(portUS);
		distanceProvider = us.getMode("Distance");
		sampleUS = new float[distanceProvider.sampleSize()];

		int buttonChoice;

		// setup odometer and start tracking
		odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		
		Display odometryDisplay = new Display(lcd);
		
		navigator = new Navigation(leftMotor, rightMotor, odometer, WHEEL_RAD, TRACK);
		distanceSensor = new USSensor(distanceProvider, sampleUS, navigator);
		navigator.setDistanceSensor(distanceSensor);
		
		Timer distancePollerTimer = new Timer(DISTANCE_POLL_PERIOD, distanceSensor);
		distancePollerTimer.start();
		
		SensorSweeper sweeper = new SensorSweeper(sweepMotor);
		sweeper.start();

		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Map 1", 0, 0);
			lcd.drawString("> Right| Map 2", 0, 1);
			lcd.drawString("^ Up   | Map 3", 0, 2);
			lcd.drawString("v Down | Map 4", 0, 3);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT
				&& buttonChoice != Button.ID_DOWN && buttonChoice != Button.ID_UP);
		
		switch (buttonChoice) {
			case Button.ID_LEFT:
				map = new int[][] {{0,2},{1,1},{2,2},{2,1},{1,0}};
				break;
			case Button.ID_RIGHT:
				map = new int[][] {{1,1},{0,2},{2,2},{2,1},{1,0}};
				break;
			case Button.ID_UP:
				map = new int[][] {{1,0},{2,1},{2,2},{0,2},{1,1}};
				break;
			case Button.ID_DOWN:
				map = new int[][] {{0,1},{1,2},{1,0},{2,1},{2,2}};
				break;
			default:
				break;
		}
		
		// start display thread
		new Thread(odometryDisplay).start();
		
		
		// used to track which way-point to head towards
		int currentWaypoint = 0;
		
		/* Main loop for moving to each waypoint.
		 * Manages specific states: the start, after reaching a waypoint, or reaching the last waypoint.
		 * 
		 */
		while (true) {
			NavigatorState navState = navigator.getNavigationState();
			
			System.out.println(navState);
			
			Thread navThread = new Thread(navigator);
			Navigation.setNavThread(navThread);
			
			if (navState == NavigatorState.start || navState == NavigatorState.navigating) {
				double destX = map[currentWaypoint][0] * TILE_SIZE;
				double destY = map[currentWaypoint][1] * TILE_SIZE;
				navigator.travelTo(destX, destY);
			}
			else if (navState == NavigatorState.atWaypoint) {
				currentWaypoint++;
				if (currentWaypoint < map.length) {
					double destX = map[currentWaypoint][0] * TILE_SIZE;
					double destY = map[currentWaypoint][1] * TILE_SIZE;
					navigator.travelTo(destX, destY);
				}
				else {
					navigator.setState(NavigatorState.end);
					break;
				}
			}
			
			navThread.start();
			
			// wait for navigation to end or if an obstacle presents itself
			try {
				navThread.join();
			} catch (InterruptedException e) {
				System.out.println("Houston we have a problem.");
			}
		}
		
//		navigator.turnTo(170);
//		navigator.turnTo(60);
//		navigator.rotateAngle(90, false);

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
