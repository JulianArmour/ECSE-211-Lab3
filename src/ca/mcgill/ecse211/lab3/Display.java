package ca.mcgill.ecse211.lab3;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.lcd.TextLCD;

/**
 * This class is used to display the content of the odometer variables (x, y,
 * Theta)
 */
public class Display implements Runnable {

    private Odometer odo;
    private TextLCD lcd;
    private double[] position;
    private final long DISPLAY_PERIOD = 500;
    private long timeout = Long.MAX_VALUE;

    /**
     * Constructs a {@link Display} object with default values.
     * 
     * @param lcd the EV3's LCD screen
     * 
     * @throws OdometerExceptions
     */
    public Display(TextLCD lcd) throws OdometerExceptions {
        odo = Odometer.getOdometer();
        this.lcd = lcd;
    }

    /**
     * 
     * @param lcd      The handle for the screen of the EV3
     * @param timeout  The period for how often the display is updated
     * @throws OdometerExceptions   
     */
    public Display(TextLCD lcd, long timeout) throws OdometerExceptions {
        odo = Odometer.getOdometer();
        this.timeout = timeout;
        this.lcd = lcd;
    }

    public void run() {

        lcd.clear();

        long updateStart, updateEnd;

        long tStart = System.currentTimeMillis();
        do {
            updateStart = System.currentTimeMillis();

            // Retrieve x, y and Theta information
            position = odo.getXYT();

            // Print x,y, and theta information
            DecimalFormat numberFormat = new DecimalFormat("######0.00");
            lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
            lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
            lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);

            // this ensures that the data is updated only once every period
            updateEnd = System.currentTimeMillis();
            if (updateEnd - updateStart < DISPLAY_PERIOD) {
                try {
                    Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        } while ((updateEnd - tStart) <= timeout);

    }

}
