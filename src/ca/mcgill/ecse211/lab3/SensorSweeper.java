package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.NXTRegulatedMotor;

public class SensorSweeper extends Thread {
    private static final int SWEEP_ANGLE = 10;
    private static final int SWEEP_SPEED = 50;
    private static final int SWEEP_ACCELERATION = 1000;
    private NXTRegulatedMotor motor;

    public SensorSweeper(NXTRegulatedMotor motor) {
        this.motor = motor;
        // System.out.println(motor.angle);
    }

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
