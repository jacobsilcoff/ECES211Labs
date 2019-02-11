package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

public class LightLocalizer extends Thread {

  /**
   * The speed of the motors during localization
   */
  private static final int MOTOR_SPEED = 100;
  /**
   * The distance the robot moves downwards before detecting the y axis, in grid units
   */
  private static final double DOWN_DIST = 0.5;

  /**
   * The time between polling the sensor, in ms
   */
  public static final int POLL_DELAY = 7;
  
  /**
   * The time waited before checking that the navigation is done
   */
  public static final int SLEEP_TIME = 50;
  /**
   * This represents the minimum difference from the mean for a light sensor reading to be
   * considered significant
   */
  private static final float LIGHT_THRESHOLD = 0.10f;

  private Odometer odo;
  private Navigation nav;
  private AveragedBuffer samples;


  /**
   * default constructor starts a navigation thread for light localizer
   */
  public LightLocalizer() throws OdometerExceptions {
    try {
      odo = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
    this.nav = new Navigation(); // start a new nav thread
    nav.start();
    samples = new AveragedBuffer();
  }

  /**
   * Move robot to (0,0) position using nav system
   * Then, it sets the heading to 0
   */
  private void movetoOrigin() {
    nav.travelTo(0, 0);
    while (nav.isNavigating()) {
      try {
        Thread.sleep(500);
      } catch (Exception e) {
      }
    }
    nav.turnTo(0);
  }

  public void run() {
    // find y
    nav.turnTo(180);
    moveToLine(false);
    odo.setY(0);
    odo.setX(0);
    
    nav.travelTo(0, -DOWN_DIST);
    while (nav.isNavigating()) {
      sleep();
    }
    nav.setSpeeds(0, 0);
    // find x
    nav.turnTo(270);
    moveToLine(false);
    odo.setX(0);

    this.movetoOrigin();
  }

  /**
   * Moves the robot forward until a line is detected
   * 
   * @param forwards True to move forward, false for backward
   */
  public void moveToLine(boolean forwards) {
    int dir = forwards ? 1 : -1;
    nav.setSpeeds(dir * MOTOR_SPEED, dir * MOTOR_SPEED);
    float[] sample = new float[Lab4.LIGHT_SENSOR.sampleSize()];
    do {
      Lab4.LIGHT_SENSOR.fetchSample(sample, 0);
      samples.add(sample[0]);
      Lab4.LCD.drawString(sample[0] + ", " + samples.getAvg(),0,4);
      sleep();
    } while (sample[0] > samples.getAvg() - LIGHT_THRESHOLD);
    nav.setSpeeds(0, 0);
  }

  /**
   * Sleeps for the default amount of time
   */
  private void sleep() {
    try {
      sleep(POLL_DELAY);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
}
