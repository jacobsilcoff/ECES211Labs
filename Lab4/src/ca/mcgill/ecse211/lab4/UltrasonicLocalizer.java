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
   * The motor speed used by the robot when trying to find the min dist
   */
  private static final int MIN_SPEED = 30;

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
    samples = new AveragedBuffer(5);
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
    Lab4.LCD.drawString("STAGE 1", 0, 4);
    double reading = readUS();
    
    while ((rising == (reading > DETECTION_DISTANCE + ERROR_THRESH)) || reading > 250) {
      sleep();
      reading = readUS();
    }
    Lab4.LCD.drawString("STAGE 2", 0, 4);
    while ((rising == (reading <= DETECTION_DISTANCE - ERROR_THRESH))
        || reading > 250) {
      sleep();
      reading = readUS();
    }
    nav.setSpeeds(0, 0);// stop

    Lab4.LCD.drawString("Edge detected", 0, 4);
 //   double lastReading = readUS();
    //if rising edge, turn other direction
    //tries to minimize value (ie, face perpendicular to wall)
//    dir = rising? -dir:dir;
//
//    nav.setSpeeds(dir * MIN_SPEED, - dir * MIN_SPEED);
//    while (lastReading >= (reading=readUS(true))) {
//      lastReading = reading;
//      sleep();
//    }
    nav.setSpeeds(0, 0);


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
    double avgAngle = (theta1 + theta2) / 2;

    if (minAngle(avgAngle, theta1) > 90) {
      avgAngle = (avgAngle + 180) % 360;
    }

    return (avgAngle + 135 + 360) % 360;
  }

  /**
   * Returns the min angle between two angles
   * @param t1 the first angle
   * @param t2 the second angle
   */
  private static double minAngle(double t1, double t2) {
    double ang = (t1 - t2 + 360) % 360;
    if (ang > 180) {
      return 360 - ang;
    } else {
      return ang;
    }
  }

  public void run() {

    //Find first edge
    double theta1 = getEdge(false);
    // switch directions and turn until another edge is detected
    double theta2 = getEdge(true);

    // turn to localized North
    // correct current theta
    double realAngle = (odo.getXYT()[2] - localizeNorth(theta1, theta2) + 360) % 360;
    odo.setXYT(0, 0, realAngle); 
    nav.turnTo(0); // turn to North/0deg
    nav.end();
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
    Lab4.LCD.drawString("US:" + (usData[0] * 100.0) + ".........", 0, 7);
    samples.add((usData[0] * 100f));
    return usData[0] * 100f;
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
