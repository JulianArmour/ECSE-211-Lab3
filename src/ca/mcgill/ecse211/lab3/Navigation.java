package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation implements Runnable {

	private static final int ROTATE_SPEED = 150;
	private static final int FORWARD_SPEED = 250;
	private static volatile NavigatorState state;
	private static Thread navThread;
	private double[] destination;
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
		this.usSensor = usSensor;
		state = NavigatorState.start;
	}

	public void travelTo(double x, double y) {
		destination[0] = x;
		destination[1] = y;
		state = NavigatorState.navigating;
	}

	public void turnTo(double theta) {
		double heading = odometer.getXYT()[2];
		// System.out.println(heading);
		double dT = ((theta - heading + 360) % 360);

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		if (dT < 180) {
			rotateAngle(dT, true);
			// leftMotor.rotate(convertAngle(wheelRadius, track, dT), true);
			// rightMotor.rotate(-convertAngle(wheelRadius, track, dT), false);
		} else {
			rotateAngle(360 - dT, false);
			// leftMotor.rotate(-convertAngle(wheelRadius, track, 360-dT), true);
			// rightMotor.rotate(convertAngle(wheelRadius, track, 360-dT), false);
		}
	}

	public void rotateAngle(double theta, boolean turnClockwise) {
		if (turnClockwise) {
			leftMotor.rotate(convertAngle(wheelRadius, track, theta), true);
			rightMotor.rotate(-convertAngle(wheelRadius, track, theta), false);
		} else {
			leftMotor.rotate(-convertAngle(wheelRadius, track, theta), true);
			rightMotor.rotate(convertAngle(wheelRadius, track, theta), false);
		}
	}

	public NavigatorState getNavigationState() {
		return state;
	}

	public boolean isNavigating() {
		return state == NavigatorState.navigating;
	}

	public void avoidObstacle() {
		state = NavigatorState.avoiding;
		navThread.interrupt();
	}

	/*
	 * run: move to destination by calling motor.rotate if reached final
	 * destination, let main thread know interrupt: stops motor and corrects
	 * 
	 */
	@Override
	public void run() {
		/*
		 * If we're in state=avoiding: 1. check left, if clear then If we're in the
		 * state=onWaypoint then calculate direction and distance, rotate, move If we're
		 * in the state=onHold then change state to navigating
		 *
		 */
		try {
			if (state == NavigatorState.navigating) {
				double[] robotPos = odometer.getXYT();
				double dx = destination[0] - robotPos[0];
				double dy = destination[1] - robotPos[1];

				double distanceToWaypoint = Math.sqrt(dx * dx + dy * dy);

				double angleToHead;

				if (dy == 0.0) {
					if (dx >= 0) {
						angleToHead = 90.0;
					} else {
						angleToHead = 270.0;
					}
				}
				// first quadrant (0-90)
				else if (dx > 0.0 && dy > 0.0) {
					angleToHead = Math.toDegrees(Math.atan(dx / dy));
				}
				// second quadrant (270-360)
				else if (dx < 0.0 && dy > 0.0) {
					angleToHead = Math.toDegrees(360 + Math.atan(dx / dy));
				}
				// third and fourth quadrant (90-270)
				else {
					angleToHead = Math.toDegrees(180 + Math.atan(dx / dy));
				}

				turnTo(angleToHead);
				
				// poll data for 0.5 second
				Thread.sleep(500);

				driveForward(distanceToWaypoint);
				
				state = NavigatorState.atWaypoint;
			}
		} catch (InterruptedException e) {
			leftMotor.stop(true);
			rightMotor.stop(false);
		}
	}

	private void driveForward(double distanceToWaypoint) {
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(wheelRadius, distanceToWaypoint), true);
		rightMotor.rotate(convertDistance(wheelRadius, distanceToWaypoint), false);
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * @param radius
	 *            radius of the wheels
	 * @param width
	 *            distance between the wheels
	 * @param angle
	 *            the angle the robot should turn
	 * @return the angle in degrees the wheels must rotate to turn the robot a
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