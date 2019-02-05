package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.NXTRegulatedMotor;

/**
 * Allows for an {@link NXTRegulatedMotor} to oscillate back and forth.
 * Effectively increasing the angle at which the sensor can detect distances
 * 
 * @author Julian Armour, Alice Kazarine
 * @version 1.0
 * @since 2019-02-01
 *
 */
public class SensorSweeper extends Thread {
    private static final int SWEEP_ANGLE = 10;
    private static final int SWEEP_SPEED = 50;
    private static final int SWEEP_ACCELERATION = 1000;
    private NXTRegulatedMotor motor;

    /**
     * 
     * @param motor The motor to be oscillated
     */
    public SensorSweeper(NXTRegulatedMotor motor) {
        this.motor = motor;
        // System.out.println(motor.angle);
    }

    /**
     * endlessly oscillates the motor back and forth
     */
    @Override
    public void run() {
        motor.setSpeed(SWEEP_SPEED);
        motor.rotate(30);
        motor.setAcceleration(SWEEP_ACCELERATION);
        while (true) {
            motor.rotate(-60);
            motor.rotate(60);
        }
    }
}
