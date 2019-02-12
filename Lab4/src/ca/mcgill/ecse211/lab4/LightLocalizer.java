package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

/**
 * This class represents a routine used by the robot
 * to find the x and y axis of the grid system it is on.
 * 
 * It uses a light sensor to detect these lines, assuming an 
 * accurate theta value stored in the odometer. 
 * @author jacob
 *
 */
public class LightLocalizer extends Thread {

  /**
   * The speed of the motors during localization
   */
  private static final int MOTOR_SPEED = 100;
  /**
   * This is the offset distance (cm) of our US sensor and the wheelbase axis
   */
  private static final double SENSOR_OFFSET = 1;
  /**
   * The distance the robot moves downwards before detecting the y axis, in grid units
   */
  private static final double DOWN_DIST = 0.5;

  /**
   * The time between polling the sensor, in ms
   */
  public static final int POLL_DELAY = 7;
  
  /**
   * The time (ms) waited before checking that the navigation is done
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
   * Creates a light localizer instance with a navigation thread
   * @throws OdometerExceptions if there are problems creating the odometer
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
   * Then, it turns to North (until the heading is 0deg)
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
  
  /**
   * Starts the thread.
   * When executed, the robot will turn to 270deg, and 
   * move backwards until the y axis is detected.
   * Then, it will move back to the center of the tile, and
   * turn 90deg down to face 180deg, moving backwards until the
   * x axis is detected. The robot will then navigate to the origin.
   */
  public void run() {
    // find y-axis
    nav.turnTo(270); //reverse robot
    moveToLine(false); //move backwards to a line (should be x=0 gridline)
    odo.setX(SENSOR_OFFSET);
    odo.setY(0);
    
    nav.travelTo(-DOWN_DIST,0); //move down to center of current block
    while (nav.isNavigating()) {
      sleep();
    }
    nav.setSpeeds(0, 0);
    
    // find x-axis
    nav.turnTo(180);
    moveToLine(false); //move backwards to a line (should be y=0 gridline)
    odo.setY(SENSOR_OFFSET);

    //both lines have been localized, can now move to origin
    this.moveForward(SENSOR_OFFSET);
    this.movetoOrigin();
  }

  /**
   * Moves the robot forward or backward until a line is detected
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
    
    Sound.beep(); //found a line
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
  
  /**
   * Moves the robot forward (straight) a certain distance, using the odometer.
   * 
   * @param dist
   */
  private void moveForward(double dist) {
    nav.setSpeeds(MOTOR_SPEED,MOTOR_SPEED);
    double[] start = nav.getOdo().getXYT();
    Lab4.LEFT_MOTOR.forward();
    Lab4.RIGHT_MOTOR.forward();
    while (Navigation.dist(nav.getOdo().getXYT(), start) < dist) {
      try {
        sleep(30);
      } catch (InterruptedException e) {
      }
    }
    nav.setSpeeds(0, 0);
  }
 
}

