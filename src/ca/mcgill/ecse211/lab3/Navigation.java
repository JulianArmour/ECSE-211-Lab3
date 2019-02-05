package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * The Navigation class handles all micro movement decisions depending on the state 
 * of the state machine. It implements {@link Runnable} so that the state is maintained
 * after the the thread terminates. 
 * 
 * @author Julian Armour, Alice Kazarine
 * @version 1.01
 * @since 2019-02-04
 */
public class Navigation implements Runnable {

    private static final int ROTATE_SPEED = 150;
    private static final int FORWARD_SPEED = 250;
    private static final int AVOIDING_DIST = 3 * 360; // how much the wheels rotate to avoid
    /** The current state that the robot is in*/
    private static volatile NavigatorState state;
    /** The handle for the thread that is running {@link #run()}. This allows the {@link USSensor} thread
     * to interrupt the {@link Navigation} thread when an obstacle is detected
     */
    private static Thread navThread;
    /** The desired waypoint coordinates*/
    private double[] destination = { 0.0, 0.0 };
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private Odometer odometer;
    private double wheelRadius;
    private double track;
    private USSensor usSensor;

    public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer,
            double wheelRadius, double track) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.odometer = odometer;
        this.wheelRadius = wheelRadius;
        this.track = track;
        state = NavigatorState.start;
    }
    
    /**
     * Sets the current waypoint that the robot should
     * travel to.
     * 
     * @param x The x-coordinate of a desired waypoint
     * @param y The y-coordinate of a desired waypoint
     */
    public void travelTo(double x, double y) {
        destination[0] = x;
        destination[1] = y;
    }
    
    /**
     * Turns the robot towards an absolute (i.e. not relative) angle
     * on the platform
     * 
     * @param theta The angle in degrees the robot will rotato to.
     */
    public void turnTo(double theta) {
        // angle component of the odometer
        double heading = odometer.getXYT()[2];
        // cyclic angle distance
        double dT = ((theta - heading + 360) % 360);

        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);

        // Turn the smallest angle
        if (dT < 180) {
            rotateAngle(dT, true);
        } else {
            rotateAngle(360 - dT, false);
        }
    }

    /**
     * Rotates the robot by the specified angle.
     * 
     * @param theta          The number of degrees to turn
     * @param turnClockwise  Specifies if the robot should turn
     *                       clockwise. If false, then the robot
     *                       will turn counter-clockwise.
     */
    public void rotateAngle(double theta, boolean turnClockwise) {
        if (turnClockwise) {
            leftMotor.rotate(convertAngle(wheelRadius, track, theta), true);
            rightMotor.rotate(-convertAngle(wheelRadius, track, theta), false);
        } else {
            leftMotor.rotate(-convertAngle(wheelRadius, track, theta), true);
            rightMotor.rotate(convertAngle(wheelRadius, track, theta), false);
        }
    }

    /**
     * 
     * @return The state of the navigator
     */
    public NavigatorState getNavigationState() {
        return state;
    }

    /**
     * 
     * @return True if in the navigating state
     */
    public boolean isNavigating() {
        return state == NavigatorState.navigating;
    }

    /**
     * When avoidObstacle is called, it sets the navigator's state to Avoiding. Then
     * interrupts the current navigation thread and stops both motors. The
     * {@link MapManager} will then start a new navigation thread to {@link #run()}
     * the avoidance algorithm.
     * 
     * @see #run()
     * @see MapManager
     */
    public void avoidObstacle() {
        state = NavigatorState.avoiding;
        navThread.interrupt();
        leftMotor.stop(true);
        rightMotor.stop(false);
    }

    /**
     * Runs in a thread to perform navigation, avoidance, rotation depending on the
     * current state of robot.
     * <p>
     * Rotating state: rotates the robot to the desired heading based on the robot's
     * current position and it's current desired {@link Navigation#destination}.
     * <p>
     * Navigating state: determines the distance from the robot to the waypoint
     * and drives the robot forward that distance. once the destination is reache,
     * move into the {@link NavigatorState#atWaypoint} state
     * <p>
     * Avoiding state: navigates the robot around an obstacle. The route it takes
     * to accomplish this depends on it's current location and angle on the platform
     */
    @Override
    public void run() {
        try {
            double[] robotPos = odometer.getXYT();

            if (state == NavigatorState.rotating) {
                double dx = destination[0] - robotPos[0];
                double dy = destination[1] - robotPos[1];

                double angleToHead;

                if (dy == 0.0) {
                    if (dx >= 0) {
                        angleToHead = 90.0;
                    } else {
                        angleToHead = 270.0;
                    }
                }
                // first quadrant (0-90)
                else if (dx >= 0.0 && dy > 0.0) {
                    angleToHead = Math.toDegrees(Math.atan(dx / dy));
                }
                // second quadrant (270-360)
                else if (dx < 0.0 && dy > 0.0) {
                    angleToHead = 360.0 + Math.toDegrees(Math.atan(dx / dy));
                }
                // third and fourth quadrant (90-270)
                else {
                    angleToHead = 180.0 + Math.toDegrees(Math.atan(dx / dy));
                }

                turnTo(angleToHead);
                
                // guard against state changes
                if (state != NavigatorState.avoiding) {
                    state = NavigatorState.navigating;
                }
            } else if (state == NavigatorState.navigating) {
                double dx = destination[0] - robotPos[0];
                double dy = destination[1] - robotPos[1];

                double distanceToWaypoint = Math.sqrt(dx * dx + dy * dy);

                // poll data for 1/4 second
                Thread.sleep(250);

                driveForward(distanceToWaypoint);

                // guard against state changes
                if (state != NavigatorState.avoiding) {
                    state = NavigatorState.atWaypoint;
                }
            } else if (state == NavigatorState.avoiding) {
                // conditions to turn right (robot is close to an edge)
                if (((robotPos[2] >= 350 || robotPos[2] <= 10) && (robotPos[0] <= 10))
                        || ((robotPos[2] >= 170 && robotPos[2] <= 190) && (robotPos[0] >= 51))
                        || ((robotPos[2] >= 80 && robotPos[2] <= 100) && (robotPos[1] >= 51))
                        || ((robotPos[2] >= 260 && robotPos[2] <= 280) && (robotPos[1] <= 10))) {

                    rotateAngle(80, true); // turn right 90 degrees
                    leftMotor.rotate(AVOIDING_DIST, true); // go straight 1400 degrees
                    rightMotor.rotate(AVOIDING_DIST, false);

                    rotateAngle(80, false);
                    leftMotor.rotate(AVOIDING_DIST / 2, true); // go straight 1400 degrees
                    rightMotor.rotate(AVOIDING_DIST / 2, false);
                    state = NavigatorState.rotating; // changes to the navigating state

                }
                // conditions to turn left (robot is close to an edge)
                else if (((robotPos[2] >= 350 || robotPos[2] <= 10) && (robotPos[0] >= 51))
                        || ((robotPos[2] >= 170 && robotPos[2] <= 190) && (robotPos[0] <= 10))
                        || ((robotPos[2] >= 80 && robotPos[2] <= 100) && (robotPos[1] <= 10))
                        || ((robotPos[2] >= 260 && robotPos[2] <= 280) && (robotPos[1] >= 51))) {

                    rotateAngle(80, false); // turn left 90 degrees

                    leftMotor.rotate(AVOIDING_DIST, true); // go straight 1400 degrees
                    rightMotor.rotate(AVOIDING_DIST, false);

                    rotateAngle(80, true);
                    leftMotor.rotate(AVOIDING_DIST / 2, true); // go straight 1400 degrees
                    rightMotor.rotate(AVOIDING_DIST / 2, false);

                    state = NavigatorState.rotating; // changes to the navigating state

                } 
                // if the robot is near the middle of the platform
                else {
                    rotateAngle(40, true);
                    // gather ~10 samples
                    Thread.sleep(1000);
                    // if the path is clear, go that way
                    if (usSensor.getFilteredDistance() >= 15) { // getfiltereddistance
                        rotateAngle(15, true);

                        leftMotor.rotate(AVOIDING_DIST, true); // go straight 1400 degrees
                        rightMotor.rotate(AVOIDING_DIST, false);

                        rotateAngle(60, false);
                        leftMotor.rotate(AVOIDING_DIST / 2, true); // go straight 1400 degrees
                        rightMotor.rotate(AVOIDING_DIST / 2, false);

                        state = NavigatorState.rotating; // changes to the navigating state
                    } 
                    // the path wasn't clear, scan left until a clear path is fount
                    else {
                        while (usSensor.getFilteredDistance() < 15) {
                            rotateAngle(15, false);
                        }
                        rotateAngle(15, false);
                        leftMotor.rotate(AVOIDING_DIST, true); // go straight 1400 degrees
                        rightMotor.rotate(AVOIDING_DIST, false);

                        rotateAngle(60, true);
                        leftMotor.rotate(AVOIDING_DIST / 2, true); // go straight 1400 degrees
                        rightMotor.rotate(AVOIDING_DIST / 2, false);

                        state = NavigatorState.rotating; // changes to the navigating state
                    }

                }
            }
        } catch (InterruptedException e) {
            // an obstacle has been detected
        }
    }

    /**
     * Causes the robot to move forward the specified distance
     * 
     * @param distance The distance in centimeters the robot should move
     */
    private void driveForward(double distance) {
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(FORWARD_SPEED);

        leftMotor.rotate(convertDistance(wheelRadius, distance), true);
        rightMotor.rotate(convertDistance(wheelRadius, distance), false);
    }

    /**
     * This method allows the conversion of a distance to the total rotation of each
     * wheel needed to cover that distance.
     * 
     * @param radius   The radius of the robot's wheels
     * @param distance The distance to be converted to degrees
     * @return The angle the wheels should turn to cover the distance
     */
    private static int convertDistance(double radius, double distance) {
        return (int) ((180.0 * distance) / (Math.PI * radius));
    }

    /**
     * @param radius
     *            The radius of the wheels
     * @param width
     *            The distance between the wheels
     * @param angle
     *            The angle the robot should turn
     * @return The angle in degrees the wheels must rotate to turn the robot a
     *         certain angle
     */
    private static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }

    public static Thread getNavThread() {
        return navThread;
    }

    public static void setNavThread(Thread navThread) {
        Navigation.navThread = navThread;
    }

    public void setDistanceSensor(USSensor distanceSensor) {
        this.usSensor = distanceSensor;
    }

    public void setState(NavigatorState state) {
        this.state = state;
    }

}
