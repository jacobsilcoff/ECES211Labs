package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

public class UltrasonicLocalizer extends Thread {

  /**
   * The time between polling the sensor, in ms
   */
  public static final int SLEEP_TIME = 15;
  /**
   * The distance (cm) below which the robot assumes it is looking at the wall
   */
  public static final double DETECTION_DISTANCE = 30.48;
  /**
   * The motor speed used by the robot when turning
   */
  private static final int ROTATE_SPEED = 100;

  /**
   * error threshold for finding falling or rising edge angles (cm)
   */
  private static final double ERROR_THRESH = 2;


  private Odometer odo;
  private Navigation nav;
  private AveragedBuffer samples;
  private Mode mode;



  /**
   * default constructor for US localizer for rising or falling edge localization
   * 
   * @param mode RISING_EDGE or FALLING_EDGE
   */
  public UltrasonicLocalizer(Mode mode) {
    this.mode = mode;
    samples = new AveragedBuffer();
    try {
      odo = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }

    // start a new nav thread
    try {
      this.nav = new Navigation();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
    nav.start();
  }


  /**
   * Turns the robot clockwise or counterclockwise
   * until a falling or rising edge is detected, as 
   * measured by the ultrasonic sensor, and then returns
   * the angle of that edge.
   * @param rising True to find the rising edge, false for falling
   * @param cw True to turn clockwise, false for counter clockwise
   * @return
   */
  public double getEdge(boolean rising, boolean cw) {
    int dir = cw? 1 : -1;
    nav.setSpeeds(dir * ROTATE_SPEED, - dir * ROTATE_SPEED);

    while (rising == readUS() > DETECTION_DISTANCE + ERROR_THRESH) {
      sleep();
    }
    while (rising == readUS() < DETECTION_DISTANCE + ERROR_THRESH) {
      sleep();
    }
    nav.setSpeeds(0, 0);// stop
    return odo.getXYT()[2];
  }

  /**
   * Turns the robot clockwise or counterclockwise
   * until an edge is detected, as measured by the ultrasonic sensor,
   * and then returns the angle of that edge. Choice of falling or
   * rising edge is based on the mode of the USLocalizer.
   * @param cw True to turn clockwise, false for counter clockwise
   * @return
   */
  public double getEdge(boolean cw) {
    return getEdge(mode == Mode.RISING_EDGE, cw);
  }


  /**
   * gets north heading
   * 
   * @param theta1 first angle detected from localization
   * @param theta2 2nd angle detected from localization
   */
  private double localizeNorth(double theta1, double theta2) {
    double thetaNorth = 0;
    double avgAngle = (theta1 + theta2) / 2;
    if (theta1 > theta2) {
      thetaNorth = 135 - avgAngle;
    } else {
      thetaNorth = 315 - avgAngle;
    }
    return thetaNorth;
  }

  public void run() {
    
    //Find first edge
    double theta1 = getEdge(false);
    // switch directions and turn until another edge is detected
    double theta2 = getEdge(true);

    // turn to localized North
    // correct current theta
    odo.setXYT(0, 0, odo.getXYT()[2] + localizeNorth(theta1, theta2)); 
    nav.turnTo(0); // turn to North/0deg
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
   * 
   * @return The US reading in cm
   */
  public float readUS() {
    float[] usData = new float[Lab4.US_SENSOR.sampleSize()];
    Lab4.US_SENSOR.fetchSample(usData, 0);
    Lab4.LCD.drawString("US:" + (usData[0] * 100.0), 0, 7);
    samples.add((int) (usData[0] * 100.0));
    return (int) (usData[0] * 100.0);
  }

  /**
   * Polls the ultrasonic sensor and returns the result,
   * which can use the averaged filter if desired
   * @param buffered True to use rolling avg filter, false to get simple reading
   * @return The US reading in cm
   */
  public float readUS(boolean buffered) {
    if (buffered) {
      readUS();
      return samples.getAvg();
    } else {
      return readUS();
    }
  }

  /**
   * This enumeration represents the different possible localization modes that the ultrasonic
   * sensor can use to align the robot with the walls.
   * 
   * @author jacob
   *
   */
  public enum Mode {
    RISING_EDGE, FALLING_EDGE;
  }

}
