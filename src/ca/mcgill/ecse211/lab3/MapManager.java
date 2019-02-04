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
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;

/**
 * Main entry point for the navigation program. MapManager allows a user to
 * choose from 4 maps. The robot will then navigate, in order, to each Waypoint.
 * MapManager keeps track of which waypoints have been visited and updates which
 * waypoint the robot should travel to next.
 * 
 * @author Julian Armour, Alice Kazarine
 * @version 1.01
 * @since 2018-02-04
 */
public class MapManager {
    /** The left and right motors of the robot. */
    private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
    /** The motor used to oscillate the ultrasonic sensor. */
    private static final NXTRegulatedMotor sweepMotor = new NXTRegulatedMotor(LocalEV3.get().getPort("D"));
    /** The display of the EV3. */
    private static final TextLCD lcd = LocalEV3.get().getTextLCD();
    /** The length of a tile. */
    public static final double TILE_SIZE = 30.48;
    /** The radius of the wheels in centimeters. */
    public static final double WHEEL_RAD = 2.1; // Radius increase = distance decrease
    /** The distance between both wheels in centimeters */
    public static final double TRACK = 10.9; // Width decrease = turn angle increase. 11.0
    /** The acceleration of the motor */
    private static final int MOTOR_ACCELERATION = 500;
    /** The time elapsed between sensor samples in milliseconds. */
    private static final int DISTANCE_POLL_PERIOD = 100;
    /** The list of waypoints for the robot to visit, represented as an array. */
    private static int[][] map;
    /** The port used for the ultrasonic sensor */
    private static Port portUS;
    /** The sensor mode of the ultrasonic sensor */
    private static SensorModes us;
    /** The interface for polling samples from the ultrasonic sensor */
    private static SampleProvider distanceProvider;
    /** The storage array for the ultrasonic sensor samples */
    private static float[] sampleUS;
    /** The class used for keeping track of the robot's position */
    private static Odometer odometer;
    /** The class responsible for gathering distance samples and filtering
     * via a median filter.
     */
    private static USSensor distanceSensor;
    /** The class that handles navigation for the robot */
    private static Navigation navigator;

    public static void main(String[] args) throws OdometerExceptions {
        int buttonChoice;
        // setup motors
        leftMotor.setAcceleration(MOTOR_ACCELERATION);
        rightMotor.setAcceleration(MOTOR_ACCELERATION);
        leftMotor.synchronizeWith(new RegulatedMotor[] {rightMotor});
        // ultrasonic sensor setup
        portUS = LocalEV3.get().getPort("S2");
        us = new EV3UltrasonicSensor(portUS);
        distanceProvider = us.getMode("Distance");
        sampleUS = new float[distanceProvider.sampleSize()];        
        // set up odometer
        odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
        Thread odoThread = new Thread(odometer);
        // start odometer
        odoThread.start();
        // display setup
        Display odometryDisplay = new Display(lcd);
        
        // set up navigation and distance sensor with median filter
        navigator = new Navigation(leftMotor, rightMotor, odometer, WHEEL_RAD, TRACK);
        distanceSensor = new USSensor(distanceProvider, sampleUS, navigator);
        // setter injection needed because both objects are dependencies of one-another
        navigator.setDistanceSensor(distanceSensor);
        
        
        Timer distancePollerTimer = new Timer(DISTANCE_POLL_PERIOD, distanceSensor);
        distancePollerTimer.start();

        SensorSweeper sweeper = new SensorSweeper(sweepMotor);
        sweeper.start();

        do {
            // clear the display
            lcd.clear();

            // ask the user whether the motors should drive in a square or float
            lcd.drawString("^ Up    | Map 1", 0, 0);
            lcd.drawString("> Right | Map 2", 0, 1);
            lcd.drawString("v Down  | Map 3", 0, 2);
            lcd.drawString("< Left  | Map 4", 0, 3);

            buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
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
            if (navState == NavigatorState.rotating) {
                double destX = map[currentWaypoint][0] * TILE_SIZE;
                double destY = map[currentWaypoint][1] * TILE_SIZE;
                navigator.travelTo(destX, destY);
            } else if (navState == NavigatorState.atWaypoint) {
                currentWaypoint++;
                if (currentWaypoint < map.length) {
                    double destX = map[currentWaypoint][0] * TILE_SIZE;
                    double destY = map[currentWaypoint][1] * TILE_SIZE;

                    navigator.setState(NavigatorState.rotating);
                    navigator.travelTo(destX, destY);
                } else {
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

        // navigator.turnTo(170);
        // navigator.turnTo(60);
        // navigator.rotateAngle(90, false);

        while (Button.waitForAnyPress() != Button.ID_ESCAPE)
            ;
        System.exit(0);
    }
}
