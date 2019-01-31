/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {
	
	public static final double RAD_TO_DEG = 57.2958; // conversion constant for conversions

	private OdometerData odoData;
	private static Odometer odo = null; // Returned as singleton

	// Motors and related variables
	// to keep track of the previous tachometer count
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private final double TRACK;
	private final double WHEEL_RAD;

	private double[] position;

	// odometer update period in ms
	private static final long ODOMETER_PERIOD = 25;

	/**
	 * This is the default constructor of this class. It initiates all motors and
	 * variables once.It cannot be accessed externally.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @throws OdometerExceptions
	 */
	private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, final double TRACK,
			final double WHEEL_RAD) throws OdometerExceptions {
		odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
													// manipulation methods
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// Reset the values of x, y and z to 0
		odoData.setXYT(0, 0, 0);

		// the current tachometer count will be used as the "origin"
		this.leftMotorTachoCount = leftMotor.getTachoCount();
		this.rightMotorTachoCount = rightMotor.getTachoCount();

		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;

	}

	/**
	 * This method is meant to ensure only one instance of the odometer is used
	 * throughout the code.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @return new or existing Odometer Object
	 * @throws OdometerExceptions
	 */
	public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
		if (odo != null) { // Return existing object
			return odo;
		} else { // create object and return it
			odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
			return odo;
		}
	}

	/**
	 * This class is meant to return the existing Odometer Object. It is meant to be
	 * used only if an odometer object has been created
	 * 
	 * @return error if no previous odometer exists
	 */
	public synchronized static Odometer getOdometer() throws OdometerExceptions {

		if (odo == null) {
			throw new OdometerExceptions("No previous Odometer exits.");

		}
		return odo;
	}

	/**
	 * This method is where the logic for the odometer will run. Use the methods
	 * provided from the OdometerData class to implement the odometer.
	 */
	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			double disL, disR, deltaD, deltaT, dX, dY;
			int nowTachoL, nowTachoR;

			updateStart = System.currentTimeMillis();

			// main algorithm to compare past position with present
			position = odo.getXYT();

			// get the current tachometer count to compare with the previous ones
			nowTachoL = leftMotor.getTachoCount();
			nowTachoR = rightMotor.getTachoCount();
			
			// the displacement of the wheels from their rotations
			disL = Math.PI * WHEEL_RAD * (nowTachoL - leftMotorTachoCount) / 180;
			disR = Math.PI * WHEEL_RAD * (nowTachoR - rightMotorTachoCount) / 180;
			
			// save the tachometer count of both wheels for later
			leftMotorTachoCount = nowTachoL;
			rightMotorTachoCount = nowTachoR;
			
			// an approximation for the displacment for the center of the robot
			deltaD = 0.5 * (disL + disR);
			
			/* an approximation for the change in heading angle for the robot
			 * since sin(x) ~=~ x for small x, this is a good approximation because
			 * the tachometer count is polled very often, relative to the speed of the robot
			 */
			deltaT = ((disL - disR) / TRACK) * RAD_TO_DEG;
			
			/* from the change in angle and displacement, 
			 * the change in X and Y components (vector) is easily obtained.
			 * Then the current position can be updated
			 */
			position[2] += deltaT; //position[2] is heading angle
			dX = deltaD * Math.sin(position[2] / RAD_TO_DEG);
			dY = deltaD * Math.cos(position[2] / RAD_TO_DEG);
			position[0] += dX; // position[0] is x-position
			position[1] += dY; // position [1] is y-position
			
			odo.update(dX, dY, deltaT);
			
			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}
	}

}
