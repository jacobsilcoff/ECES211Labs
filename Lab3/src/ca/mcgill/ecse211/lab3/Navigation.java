package ca.mcgill.ecse211.lab3;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/*
 * A class used to navigate the robot according to waypoints and obstacles.
 */
public class Navigation extends Thread{


  private static final int FORWARD_SPEED = 250;		//forward speed (deg/s)
  private static final int ROTATE_SPEED = 100;		//turning speed (deg/s)
  private static final double TILE_SIZE = 30.48;		//grid spacing (cm)
  private static final double DIST_THRESH = 1;		//distance threshold (cm)
  public static final double EMERGENCY_THRESH = 17;	//emergency threshold for US sensor obstacle threshold distance (cm)
  private static final double T_THRESH = 0.5;			//turning angle threshold (deg)

  private static final int SLEEP_TIME = 30;			//sleep cycle (ms)

  private final double lRad;
  private final double rRad;
  private final double track;
  private boolean isNavigating;
  private Odometer odo;

  private static int numInstructions = 0;
  private double destX;
  private double destY;
  private double destT;

  /**
   * default constructor for navigation (called in Lab3.java)
   * @param left 		left motor
   * @param right 	right motor
   * @param odometer	odometer instance
   * @param leftRadius	left wheel radius
   * @param rightRadius	right wheel radius
   * @param track			track / wheelbase 
   * @param Lab3.US_SENSOR		US sensor sample provider
   */
  public Navigation(Odometer odometer) {
    odo = odometer;
    this.track = Lab3.TRACK;
    lRad = Lab3.WHEEL_RAD;
    rRad = Lab3.WHEEL_RAD;
    isNavigating = false;
    destX = destY = destT = 0; //default destination is (0,0,0)
  }

  /**
   * Sets robot to travel to a given TILE point,
   * updating the destination direction and position
   * @param x The desired x in cm
   * @param y The desired y in cm
   */
  public void travelTo(double x, double y) {
    destX = x*TILE_SIZE; //convert tile point to destination X coord (cm)
    destY = y*TILE_SIZE; //convert Y tile pt
    updateT();
    isNavigating = true;
    Lab3.LCD.drawString("Dest:" + (int)destX +"," +(int)destY +"," +(int)destT, 0, 4);

  }

  public Odometer getOdo() {
    return odo;
  }

  

  /**
   * This method checks isNavigating variable
   * @return true if another thread has called travelTo() or turnTo() and has yet to return; otherwise false
   */
  public boolean isNavigating() {
    return isNavigating;
  }


  /**
   * Turns the robot on the spot to a given angle theta using the MINIMUM angle
   * @param theta The desired absolute angle in degrees
   * @param speed The turning speed
   */
  public void turnTo(double theta, int speed) {
    double presTheta = odo.getXYT()[2]; //get current heading
    double ang = (theta - presTheta + 360) % 360; //gets absolute angle required to turn
    Lab3.LEFT_MOTOR.setSpeed(speed);
    Lab3.RIGHT_MOTOR.setSpeed(speed);

    //turn using MINIMUM angle
    if (ang < 180) {
      Lab3.LCD.drawString("Ang: " + ang + "deg  ", 0, 5); //display angle of rotation
      //increase angle
      Lab3.LEFT_MOTOR.rotate(convertAngle(lRad, track, ang), true);
      Lab3.RIGHT_MOTOR.rotate(-convertAngle(rRad, track, ang), false);//make false for rough answer
    } else {
      ang = 360 - ang; 
      Lab3.LCD.drawString("Ang: " + ang+"deg   ", 0, 5); //display angle of rotation
      //Need to check against odometer
      Lab3.LEFT_MOTOR.rotate(-convertAngle(lRad, track, ang), true);
      Lab3.RIGHT_MOTOR.rotate(convertAngle(rRad, track, ang), false);
    }
    updateT();//update new angle after turn;
  }

  /**
   * Turns the robot on the spot to a given angle theta at a default speed
   * @param theta The desired absolute angle in degrees
   * @param speed The turning speed
   */
  public void turnTo(double theta) {
    turnTo(theta, ROTATE_SPEED);
  }


  enum State{INIT,TURNING,TRAVELING, EMERGENCY}

  /**
   * Implements a state machine of initializing, turning
   * traveling, or handling an emergency obstacle
   */
  public void run() {
    State state = State.INIT;

    ObstacleAvoidance avoidance = new ObstacleAvoidance(this);
    while (true) {
      switch(state) {
        case INIT:
          Lab3.LCD.drawString("State: INIT", 0, 6); //display state
          if (isNavigating) {
            state = State.TURNING;
          }
          break;
        case TURNING:
          Lab3.LCD.drawString("State: TURN", 0, 6);
          turnTo(destT); //turn robot to destination theta
          if (facing(destT)) { //if turned, start travel
            state = State.TRAVELING;
          }
          break;
        case TRAVELING:
          Lab3.LCD.drawString("State: TRVL", 0, 6);
          //make sure desired distance is correct
          updateT();
          if (checkEmergency()) { //obstacle detected
            state = State.EMERGENCY;
            avoidance = new ObstacleAvoidance(this);
            avoidance.start(); //start avoidance
          }
          else if (!facing(destT)) {
            state = State.TURNING; //re-check heading and finish turning
          }
          else if (!checkIfDone()) {
            updateTravel(); //finish traveling
            if (!facing(destT)){ //check heading again
              updateT();
              turnTo(destT);
            }
          } else { //Arrived
            setSpeeds(0,0); //stop
            isNavigating = false; //finished traveling
            state = State.INIT; //return to initialize case
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
      } catch (InterruptedException e) {}
    }
  }

  /**
   * Polls the US sensor
   * @return The US reading in cm
   */
  public float readUS() {
    float[] usData = new float[Lab3.US_SENSOR.sampleSize()];
    Lab3.US_SENSOR.fetchSample(usData, 0);
    Lab3.LCD.drawString("US:" +(usData[0] * 100.0), 0, 7);
    return (int) (usData[0] * 100.0);
  }

  /**
   * Slows the motor speeds when nearing destination (<20cm)
   * 
   */
  private void updateTravel() {
    double dist = dist(new double[] {destX,destY}, odo.getXYT());
    //slows down upon nearing destination
    if (dist > 20) {
      setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    } else {
      float motorSpeeds = (float)(dist/20 * (FORWARD_SPEED - 50) + 50); //slower speed proportional to distance to dest
      setSpeeds(motorSpeeds, motorSpeeds);
    }
    Lab3.LEFT_MOTOR.forward();
    Lab3.RIGHT_MOTOR.forward();
  }

  /**
   * gets distance from current position to destination
   */
  public double getdist() {
    return dist(new double[] {destX,destY}, odo.getXYT());
  }

  public double getDestT() {
    return destT;
  }


  /**
   * Updates the destT (heading) to reflect
   * the real position of the robot
   */
  private void updateT() {
    double dx = destX - odo.getXYT()[0];
    double dy = destY - odo.getXYT()[1];
    if (dy == 0) {
      destT = (dx > 0) ? 90 : 270;
    } else {
      destT = Math.toDegrees(Math.atan(dx/dy)) 
          + ((dy > 0)? 0 : 180);
      destT = (destT + 360) % 360; //normalize theta
    }
  }

  public boolean checkEmergency() {
    return readUS() < EMERGENCY_THRESH;
  }

  /**
   * Checks if the robot is facing a certain angle
   * @param ang The angle to check
   */
  private boolean facing(double ang) {
    double diff = Math.abs(odo.getXYT()[2] - (ang + 360)%360);
    diff = (diff + 360) % 360;
    return (diff < T_THRESH) || ((360 - diff) < T_THRESH);
  }

  private boolean checkIfDone() {
    return dist(odo.getXYT(), new double[]{destX,destY}) < DIST_THRESH;
  }

  public void setSpeeds(float l, float r) {
    Lab3.LEFT_MOTOR.setSpeed(l);
    Lab3.RIGHT_MOTOR.setSpeed(r);
  }
  private void setSpeeds(int l, int r) {
    Lab3.LEFT_MOTOR.setSpeed(l);
    Lab3.RIGHT_MOTOR.setSpeed(r);
  }

  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }


  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  /**
   * Gets distance (cm) between two coordinates
   * @param a position array 1
   * @param b position array 2
   * @return distance (cm)
   */
  public static double dist(double[] a, double[] b) {
    if (a.length < 2 || b.length < 2) {
      return -1;
    }
    return Math.sqrt(Math.pow(a[0]-b[0], 2) + Math.pow(a[1]-b[1],2)); //minimum distance formula
  }

}
