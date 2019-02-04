package ca.mcgill.ecse211.lab3;

/**
 * This class represents a routine that can be used
 * by the robot to avoid hitting an obstacle. 
 * @author Team 71
 */
public class ObstacleAvoidance extends Thread{

  /**
   * The distance before which an object is considered an emergency
   */
  public static final int SAFE_DIST = 70;
  /**
   * The extra distance past the obstacle used to give the robot
   * room to turn
   */
  public static final int EXTRA_DIST = 10;
  public static final int TURN_SPD = 50;

  private Navigation nav;
  private boolean safe;



  public ObstacleAvoidance(Navigation nav) {
    this.nav = nav;
    safe = false;
  }

  public void run() {
    //Obstacle avoidance algorithm:
    double startAngle = nav.getOdo().getXYT()[2];

    //1. turn until edge of obstacle is detected
    double edgeDist = findEdge();

    //2. turn an extra angle to accommodate robot girth + move forward to clear the edge
    double extraT = Math.toDegrees(Math.asin(Lab3.TRACK/2/edgeDist));
    nav.turnTo(nav.getOdo().getXYT()[2] + extraT, TURN_SPD);
    moveForward(edgeDist + EXTRA_DIST);


    nav.turnTo(startAngle);
    moveForward(15);
    nav.turnTo((startAngle + 270) % 360);

    edgeDist = findEdge();
    extraT = Math.toDegrees(Math.asin(Lab3.TRACK/2/edgeDist));
    nav.turnTo(nav.getOdo().getXYT()[2] + extraT, TURN_SPD);
    moveForward(edgeDist + EXTRA_DIST);


    //two clearances made, should be safe
    safe = true;


  }

  /**
   * Turns robot until the edge of the obstacle is detected,
   * then returns the distance of that edge.
   * @return The distance from the robot to the edge. -1 if no obstacle.
   */
  private double findEdge() {
    double dist = nav.readUS();
    double edgeDist = -1;

    nav.setSpeeds(TURN_SPD, TURN_SPD);

    Lab3.LEFT_MOTOR.forward();
    Lab3.RIGHT_MOTOR.backward();  
    while ((dist = nav.readUS()) < SAFE_DIST) { //turn until obstacle is not detected
      edgeDist = dist;
      try {
        sleep(30);
      } catch (Exception e) {}
    }
    
    return edgeDist;
  }

  /**
   * Moves the robot forward (straight) a certain distance. Used in obstacle avoidance
   * @param dist
   */
  private void moveForward(double dist) {
    nav.setSpeeds(TURN_SPD * 2, TURN_SPD * 2);
    double[] start = nav.getOdo().getXYT();
    Lab3.LEFT_MOTOR.forward();
    Lab3.RIGHT_MOTOR.forward();
    while (Navigation.dist(nav.getOdo().getXYT(), start) < dist) {
      try {
        sleep(30);
      } catch (InterruptedException e) {}
    }
    nav.setSpeeds(0,0);
  }

  public boolean isResolved() {
    return safe;
  }
}
