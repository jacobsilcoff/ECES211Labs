package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

import ca.mcgill.ecse211.odometer.OdometryCorrection;


/*
 * A class used to navigate the robot according to waypoints and obstacles.
 */
public class Navigation extends Thread {
  /**
   * The motor speed of the robot when moving forward
   */
  private static final int FORWARD_SPEED = 250;
  /**
   * The motor speed used by the robot when turning
   */
  private static final int ROTATE_SPEED = 100;
  /**
   * The distance, in cm, between grid lines
   */
  private static final double TILE_SIZE = 30.48;
  /**
   * The maximum distance between two points where they are considered to be roughly equal.
   */
  private static final double DIST_THRESH = 0.5;
  /**
   * The distance at which an object is considered to be close enough to the robot to initiate the
   * emergency obstacle avoidance sequence
   */
  public static final double EMERGENCY_THRESH = 17;
  /**
   * The max difference between two angles where they are considered to be roughly equal
   */
  private static final double T_THRESH = 0.8;

  /**
   * The amount of time, in ms, that the thread will sleep for in between cycles
   */
  private static final int SLEEP_TIME = 20;

  /**
   * The distance after which the robot will no longer attempt to update its heading
   */
  private static final int CORRECTION_DIST = 4;

  /**
   * The distance were the robot starts to slow down
   */
  private static final int SLOW_DIST = 5;
  /**
   * The slowest the robot will spin its motors
   */
  private static final int MIN_SPEED = 50;


  private boolean isNavigating;
  private Odometer odo;
  private double destX;
  private double destY;
  private double destT;
  private OdometryCorrection correction;

  /**
   * Default constructor for navigation (called in Lab3.java)
   * 
   * @param correction The odometer correction that is used for the robot
   * @throws OdometerExceptions
   */
  public Navigation(OdometryCorrection correction) throws OdometerExceptions {
    odo = Odometer.getOdometer();
    isNavigating = false;
    destX = destY = destT = 0;
    this.correction = correction;
  }

  /**
   * Sets robot to travel to a given TILE point, updating the destination direction and position
   * 
   * @param x The desired x in cm
   * @param y The desired y in cm
   */
  public void travelTo(double x, double y) {
    destX = x * TILE_SIZE; // convert tile point to destination X coord (cm)
    destY = y * TILE_SIZE; // convert Y tile pt
    updateT();
    isNavigating = true;
    Lab3.LCD.drawString("Dest:" + (int) destX + "," + (int) destY + "," + (int) destT, 0, 4);

  }

  /**
   * Returns the odometer corresponding to this navigation thread. Although this method is
   * technically redundant due to the fact that Odometer is represented as a singleton, it is useful
   * to keep this in place in case we want to modify odometer later (say, to have another odometer
   * keeping track of a different set of motors)
   * 
   * @return The odometer measuring the robots position
   */
  public Odometer getOdo() {
    return odo;
  }



  /**
   * This method checks isNavigating variable
   * 
   * @return true if another thread has called travelTo() or turnTo() and has yet to return;
   *         otherwise false
   */
  public boolean isNavigating() {
    return isNavigating;
  }


  /**
   * Turns the robot on the spot to a given angle theta using the MINIMUM angle
   * 
   * @param theta The desired absolute angle in degrees
   * @param speed The turning speed
   */
  public void turnTo(double theta, int speed) {
    correction.setOn(false);
    double presTheta = odo.getXYT()[2]; // get current heading
    double ang = (theta - presTheta + 360) % 360; // gets absolute angle required to turn
    Lab3.LEFT_MOTOR.setSpeed(speed);
    Lab3.RIGHT_MOTOR.setSpeed(speed);

    // turn using MINIMUM angle
    if (ang < 180) {
      Lab3.LCD.drawString("Ang: " + ang + "deg  ", 0, 5);
      // increase angle
      Lab3.LEFT_MOTOR.rotate(convertAngle(ang), true);
      Lab3.RIGHT_MOTOR.rotate(-convertAngle(ang), false);
    } else {
      ang = 360 - ang;
      Lab3.LCD.drawString("Ang: " + ang + "deg   ", 0, 5); // display angle of rotation
      // Need to check against odometer
      Lab3.LEFT_MOTOR.rotate(-convertAngle(ang), true);
      Lab3.RIGHT_MOTOR.rotate(convertAngle(ang), false);
    }
    updateT();// update new angle after turn;
  }

  /**
   * Turns the robot on the spot to a given angle theta at a default speed
   * 
   * @param theta The desired absolute angle in degrees
   * @param speed The turning speed
   */
  public void turnTo(double theta) {
    turnTo(theta, ROTATE_SPEED);
  }


  /**
   * An enumeration of states the robot can be in as it navigates from point to point
   * 
   * @author jacob silcoff
   */
  enum State {
    INIT, TURNING, TRAVELING, EMERGENCY
  }

  /**
   * Implements a state machine of initializing, turning traveling, or handling an emergency
   * obstacle
   */
  @Override
  public void run() {
    State state = State.INIT;
    ObstacleAvoidance avoidance = new ObstacleAvoidance(this);
    while (true) {
      double[] lastPos = {0, 0};
      switch (state) {
        case INIT:
          Lab3.LCD.drawString("State: INIT", 0, 6);
          if (isNavigating) {
            state = State.TURNING;
          }
          correction.setOn(false);
          break;
        case TURNING:
          Lab3.LCD.drawString("State: TURN", 0, 6);
          turnTo(destT);
          if (facing(destT)) {
            state = State.TRAVELING;
            lastPos = odo.getXYT();
          }
          break;
        case TRAVELING:
          Lab3.LCD.drawString("State: TRVL", 0, 6);
          updateT();
          if (checkEmergency()) { // obstacle detected
            state = State.EMERGENCY;
            avoidance = new ObstacleAvoidance(this);
            avoidance.start();
          } else if (getDist() > CORRECTION_DIST && dist(lastPos, odo.getXYT()) > CORRECTION_DIST
              && !facing(destT)) {
            // re-check heading and finish turning
            state = State.TURNING;
          } else if (!checkIfDone()) {
            correction.setOn(true);
            updateTravel();
          } else { // Arrived
            setSpeeds(0, 0); // stop
            isNavigating = false; // finished traveling
            state = State.INIT; // return to initialize case
          }
          break;
        case EMERGENCY:
          Lab3.LCD.drawString("State: EMRG", 0, 6);
          if (avoidance.isResolved()) {
            state = State.TURNING;
          }
          break;
      }
      try {
        sleep(SLEEP_TIME);
      } catch (InterruptedException e) {
      }
    }
  }

  /**
   * Polls the ultrasonic sensor and returns the result
   * 
   * @return The US reading in cm
   */
  public float readUS() {
    float[] usData = new float[Lab3.US_SENSOR.sampleSize()];
    Lab3.US_SENSOR.fetchSample(usData, 0);
    Lab3.LCD.drawString("US:" + (usData[0] * 100.0), 0, 7);
    return (int) (usData[0] * 100.0);
  }

  /**
   * Slows the motor speeds when nearing destination (<20cm)
   */
  private void updateTravel() {
    double dist = getDist();
    // slows down upon nearing destination
    if (dist > DIST_THRESH) {
      double speed = FORWARD_SPEED;
      if (dist < SLOW_DIST) {
        speed = dist / SLOW_DIST * (FORWARD_SPEED - MIN_SPEED) + MIN_SPEED;
      }
      setSpeeds((float) speed, (float) speed);
    } else {
      setSpeeds(0, 0);
    }
    Lab3.LEFT_MOTOR.forward();
    Lab3.RIGHT_MOTOR.forward();
  }

  /**
   * Gets distance from current position to destination
   * 
   * @return The distance from the current position to the destination, in cm
   */
  public double getDist() {
    return dist(new double[] {destX, destY}, odo.getXYT());
  }

  /**
   * Gets the angle from the robot to the destination
   * 
   * @return the angle from the robot to the destination, in cm
   */
  public double getDestT() {
    return destT;
  }


  /**
   * Updates the destT (heading) to reflect the real position of the robot
   */
  private void updateT() {
    double dx = destX - odo.getXYT()[0];
    double dy = destY - odo.getXYT()[1];
    if (dy == 0) {
      destT = (dx > 0) ? 90 : 270;
    } else {
      destT = Math.toDegrees(Math.atan(dx / dy)) + ((dy > 0) ? 0 : 180);
      destT = (destT + 360) % 360; // normalize theta
    }
  }

  /**
   * Checks that the robot is in an emergency state by polling the ultrasonic sensor
   * 
   * @return True if an emergency is detected, false otherwise
   */
  public boolean checkEmergency() {
    return readUS() < EMERGENCY_THRESH;
  }

  /**
   * Checks if the robot is facing a certain angle
   * 
   * @param ang The angle to check
   * @return True if the robot is facing the given angle, false otherwise
   */
  private boolean facing(double ang) {
    double diff = Math.abs(odo.getXYT()[2] - (ang + 360) % 360);
    diff = (diff + 360) % 360;
    return (diff < T_THRESH) || ((360 - diff) < T_THRESH);
  }

  /**
   * Checks if the robot has arrived at its destination
   * 
   * @return True if the robot has arrived, false otherwise
   */
  private boolean checkIfDone() {
    return dist(odo.getXYT(), new double[] {destX, destY}) < DIST_THRESH;
  }

  /**
   * Sets the speeds of both motors
   * 
   * @param l The desired speed of the left motor
   * @param r The desired speed of the right motor
   */
  public void setSpeeds(float l, float r) {
    Lab3.LEFT_MOTOR.setSpeed(l);
    Lab3.RIGHT_MOTOR.setSpeed(r);
  }

  /**
   * Sets the speeds of both motors
   * 
   * @param l The desired speed of the left motor
   * @param r The desired speed of the right motor
   */
  private void setSpeeds(int l, int r) {
    Lab3.LEFT_MOTOR.setSpeed(l);
    Lab3.RIGHT_MOTOR.setSpeed(r);
  }


  /**
   * Takes a given distance, and returns the number of degrees the robot's wheels need to turn to
   * move forward that distance.
   * 
   * @param distance The distance the robot moves forward, in cm
   * @return The number of degrees of wheel rotation needed for the given distance
   */
  private static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * Lab3.WHEEL_RAD));
  }


  /**
   * Takes a given angle, and returns the number of degrees the robot's wheels need to turn for the
   * entire robot to rotate that angle.
   * 
   * @param angle The desired angle for the robot to turn, in degrees
   * @return The number of degrees of wheel rotation needed for the given angle
   */
  private static int convertAngle(double angle) {
    return convertDistance(Math.PI * Lab3.TRACK * angle / 360.0);
  }

  /**
   * Gets distance (cm) between two coordinates
   * 
   * @param a position array 1 where a[0] is its x, and a[1] is its y
   * @param b position array 2 where b[0] is its x, and b[1] is its y
   * @return The distance between a and b, in cm
   */
  public static double dist(double[] a, double[] b) {
    if (a.length < 2 || b.length < 2) {
      return -1;
    }
    return Math.sqrt(Math.pow(a[0] - b[0], 2) + Math.pow(a[1] - b[1], 2)); // minimum distance
                                                                           // formula
  }

}
