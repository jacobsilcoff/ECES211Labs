package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

public class UltrasonicLocalizer extends Thread {
  
  /**
   * The time between polling the sensor,
   * in ms
   */
  public static final int SLEEP_TIME = 15;
  /**
   * The distance below which the robot assumes
   * it is looking at the wall
   */
  public static final double DETECTION_DISTANCE = 100;
  /**
   * The motor speed used by the robot when turning
   */
  private static final int ROTATE_SPEED = 100;
  
  private Odometer odo;
  private AveragedBuffer samples;
  private Mode mode;
  
  public UltrasonicLocalizer(Mode mode) {
    this.mode = mode;
    samples = new AveragedBuffer();
    try {
      odo = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
  }
  
  
  public void run() { 
    
    if (mode == Mode.FALLING_EDGE) {
      //turns until falling edge is detected
      //switch directions
      //turn until another falling edge is detected
      //average the two to find 45 deg
    } else {
      //turns until rising edge is detected
      //switch directions
      //turn until another rising edge is detected
      //average the two to find position
    }
    
  }
  
  /**
   * Sleeps for the default amount of time
   */
  private void sleep() {
    try {
      sleep(SLEEP_TIME);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
  
  /**
   * Polls the ultrasonic sensor and returns the result
   * @return The US reading in cm
   */
  public float readUS() {
    float[] usData = new float[Lab4.US_SENSOR.sampleSize()];
    Lab4.US_SENSOR.fetchSample(usData, 0);
    Lab4.LCD.drawString("US:" +(usData[0] * 100.0), 0, 7);
    return (int) (usData[0] * 100.0);
  }
  
  /**
   * This enumeration represents the different
   * possible localization modes that the
   * ultrasonic sensor can use to align the robot
   * with the walls.
   * @author jacob
   *
   */
  public enum Mode {
    RISING_EDGE, FALLING_EDGE;
  }

}
