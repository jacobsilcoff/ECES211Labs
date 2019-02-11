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
  private static final double ERROR_THRES = 2;
  

  private Odometer odo;
  private Navigation nav;
  private AveragedBuffer samples;
  private Mode mode;

  private double theta1; // first localization angle
  private double theta2; // second localization angle
  private double thetaNorth; // localization of 0deg

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
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    nav.start();
  }


  /**
   * turns robot COUNTERCLOCKWISE to find 1st falling edge
   * @return double theta1;
   */
  private double getFirstFalling() {
   
    while(readUS() < DETECTION_DISTANCE +ERROR_THRES) {
      nav.setSpeeds(ROTATE_SPEED,-ROTATE_SPEED);
    }
    while(readUS() > DETECTION_DISTANCE) {
      nav.setSpeeds(ROTATE_SPEED,-ROTATE_SPEED);
    }
    
    nav.setSpeeds(0,0);//stops within the error threshold
    return odo.getXYT()[2];
  }

  /**
   * turns robot COUNTERCLOCKWISE to find 2nd falling edge
   */
  private double getSecondFalling() {
    while(readUS() < DETECTION_DISTANCE +ERROR_THRES) {
      nav.setSpeeds(-ROTATE_SPEED,ROTATE_SPEED);
    }
    while(readUS() > DETECTION_DISTANCE +ERROR_THRES) {
      nav.setSpeeds(-ROTATE_SPEED,ROTATE_SPEED);
    }
    
    nav.setSpeeds(0,0);//stop
    return odo.getXYT()[2];
  }

  /**
   * turns robot COUNTERCLOCKWISE to find 1st rising edge
   */
  private double getFirstRising() {
    while(readUS() > DETECTION_DISTANCE +ERROR_THRES) {
      nav.setSpeeds(-ROTATE_SPEED,ROTATE_SPEED);
    }
    while(readUS() < DETECTION_DISTANCE +ERROR_THRES) {
      nav.setSpeeds(-ROTATE_SPEED,ROTATE_SPEED);
    }
    
    nav.setSpeeds(0,0);//stop
    return odo.getXYT()[2];
  }
  
  /**
   * turns robot COUNTERCLOCKWISE to find 2nd rising edge
   */
  private double getSecondRising() {
    while(readUS() > DETECTION_DISTANCE +ERROR_THRES) {
      nav.setSpeeds(ROTATE_SPEED,ROTATE_SPEED);
    }
    while(readUS() < DETECTION_DISTANCE +ERROR_THRES) {
      nav.setSpeeds(ROTATE_SPEED,ROTATE_SPEED);
    }
    
    nav.setSpeeds(0,0);//stop
    return odo.getXYT()[2];
  }

  
  /**
   * gets north heading
   * @param theta1 first angle detected from localization
   * @param theta2 2nd angle detected from localization
   */
  private void localizeNorth(double theta1, double theta2){
   
  }
  
  public void run() {

    if (mode == Mode.FALLING_EDGE) {
      // turns until falling edge is detected
      theta1 = getFirstFalling();
      // switch directions and turn until another falling edge is detected
      theta2 = getSecondFalling();
      
    } else {
      // turns until rising edge is detected
      theta1 = getFirstRising();
      // switch directions + turn until another rising edge is detected
      theta2 = getSecondRising();
    }

    //now we have the 1st and 2nd angles, get 0deg/north heading
    localizeNorth(theta1,theta2);
    
    //turn to localized North
    odo.setXYT(0, 0, odo.getXYT()[2]+ thetaNorth); //correct current theta
    nav.turnTo(0); //turn to North/0deg
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
    return (int) (usData[0] * 100.0);
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
