// Lab2.java
package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;

/**
 * Main entry point for the navigation program. MapManager allows a user to
 * choose from 4 maps. The robot will then navigate, in order, to each Waypoint.
 * MapManager keeps track of which waypoints have been visited and updates which
 * waypoint the robot should travel to next.
 * 
 * @author Julian Armour, Alice Kazarine
 */
public class MapManager {
    /** The array of map waypoints */
    private static int[][] map;
    /** The left motor of the robot.*/
    private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    /** The right motor of the robot.*/
    private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
    /** The motor used to oscillate the ultrasonic sensor.*/
    private static final NXTRegulatedMotor sweepMotor = new NXTRegulatedMotor(LocalEV3.get().getPort("D"));
    /** The EV3's screen.*/
    private static final TextLCD lcd = LocalEV3.get().getTextLCD();
    /** The tile's length.*/
    public static final double TILE_SIZE = 30.48;
    /** The radius of the wheel.*/
    public static final double WHEEL_RAD = 2.1; // Radius increase = distance decrease
    /** The length of the track.*/
    public static final double TRACK = 11.0; // Width decrease = turn angle increase
    /** The acceleration of the motors*/
    private static final int MOTOR_ACCELERATION = 1000;
    /** The period between ultrasensor samples*/
    private static final int DISTANCE_POLL_PERIOD = 100;

    // defined port and sensor
    private static Port portUS;
    private static SensorModes us;
    /** The interface for the ultrasonic sensor*/
    private static SampleProvider distanceProvider;
    /** The array for storing samples from the ultrasonic sensor*/
    private static float[] sampleUS;
    /** The class that tracks the position and heading of the robot*/
    private static Odometer odometer;
    /** The class that drives the ultrasonic sensor and filters the data using a median filter*/
    private static USSensor distanceSensor;
    /** The class that handles all navigation requests and robot movements*/
    private static Navigation navigator;

    public static void main(String[] args) throws OdometerExceptions {
        leftMotor.setAcceleration(MOTOR_ACCELERATION);
        rightMotor.setAcceleration(MOTOR_ACCELERATION);
        
        // setup ultrasonic sensor
        portUS = LocalEV3.get().getPort("S2");
        us = new EV3UltrasonicSensor(portUS);
        distanceProvider = us.getMode("Distance");
        sampleUS = new float[distanceProvider.sampleSize()];
        
        // which button was pressed at the begining of the program
        int buttonChoice;
        
        // setup odometer and start tracking
        odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
        Thread odoThread = new Thread(odometer);
        odoThread.start();
        
        // prepare the display for the odometer
        Display odometryDisplay = new Display(lcd);

        // set up navigator and filtered ultrasonic sensor
        navigator = new Navigation(leftMotor, rightMotor, odometer, WHEEL_RAD, TRACK);
        distanceSensor = new USSensor(distanceProvider, sampleUS, navigator);
        // setter injection to deal with mutual dependency
        navigator.setDistanceSensor(distanceSensor);

        // create and start a timer for polling distances and filtering
        Timer distancePollerTimer = new Timer(DISTANCE_POLL_PERIOD, distanceSensor);
        distancePollerTimer.start();
        
        // start the oscillation for the motor that holds the ultrasonic sensor
        SensorSweeper sweeper = new SensorSweeper(sweepMotor);
        sweeper.start();

        do {
            // clear whatever is on the screen
            lcd.clear();

            // Menu for choosing which map to navigate
            lcd.drawString("^ Up    | Map 1", 0, 0);
            lcd.drawString("> Right | Map 2", 0, 1);
            lcd.drawString("v Down  | Map 3", 0, 2);
            lcd.drawString("< Left  | Map 4", 0, 3);

            buttonChoice = Button.waitForAnyPress();
        } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_DOWN
                && buttonChoice != Button.ID_UP);

        // The four different maps to choose from
        switch (buttonChoice) {
        case Button.ID_UP:
            map = new int[][] { { 0, 2 }, { 1, 1 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
            break;
        case Button.ID_RIGHT:
            map = new int[][] { { 1, 1 }, { 0, 2 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
            break;
        case Button.ID_DOWN:
            map = new int[][] { { 1, 0 }, { 2, 1 }, { 2, 2 }, { 0, 2 }, { 1, 1 } };
            break;
        case Button.ID_LEFT:
            map = new int[][] { { 0, 1 }, { 1, 2 }, { 1, 0 }, { 2, 1 }, { 2, 2 } };
            break;
        default:
            break;
        }

        // start display thread
        new Thread(odometryDisplay).start();

        // used to track which way-point to head towards
        int currentWaypoint = 0;

        /*
         * Main loop for moving to each waypoint. Manages specific states: the start,
         * after reaching a waypoint, or reaching the last waypoint.
         */
        while (true) {
            NavigatorState navState = navigator.getNavigationState();

            Thread navThread = new Thread(navigator);
            Navigation.setNavThread(navThread);

            /*
             * at the starting state, change to rotating state. This is done to provide a
             * human-understandable entry point.
             */
            if (navState == NavigatorState.start) {
                navigator.setState(NavigatorState.rotating);
                navState = NavigatorState.rotating;
            }
            // if the robot detects an obstacle
            if (navState == NavigatorState.rotating) {
                double destX = map[currentWaypoint][0] * TILE_SIZE;
                double destY = map[currentWaypoint][1] * TILE_SIZE;
                navigator.travelTo(destX, destY);
            } 
            // if the robot is on a waypoint, load the next waypoint in the array
            else if (navState == NavigatorState.atWaypoint) {
                currentWaypoint++;
                if (currentWaypoint < map.length) {
                    double destX = map[currentWaypoint][0] * TILE_SIZE;
                    double destY = map[currentWaypoint][1] * TILE_SIZE;

                    navigator.setState(NavigatorState.rotating);
                    navigator.travelTo(destX, destY);
                } 
                // when there aren't any more waypoints, move into terminating state
                else {
                    navigator.setState(NavigatorState.end);
                    break;
                }
            }
            /* after macro level waypoint management, let the navigator handle
             * the micro movements of how to get to the waypoint, or avoid an
             * obstacle.
             */
            navThread.start();

            // wait for navigation to end or if an obstacle presents itself
            try {
                navThread.join();
            } catch (InterruptedException e) {
                System.out.println("Main thread was interrupted");
            }
        }

        while (Button.waitForAnyPress() != Button.ID_ESCAPE);
        System.exit(0);
    }
}
