package ca.mcgill.ecse211.lab3;

/**
 * This class represents a routine that can be used
 * by the robot to avoid hitting an obstacle. 
 * @author Team 71
 */
public class ObstacleAvoidance extends Thread{

  /**
   * This represents the closest an object can be before
   * it is considered to be an emergency.
   */
  public static final int SAFE_DIST = 70;
  /**
   * The extra distance past the obstacle used to give the robot
   * room to turn
   */
  public static final int EXTRA_DIST = 10;
  /**
   * The motor speed used by the robot when turning
   */
  public static final int TURN_SPD = 50;

  private Navigation nav;
  private boolean safe;


  /**
   * Creates an ObstacleAvoidance instance given a
   * Navigation process. 
   * @param nav The navgiation thread being used to control the robot
   */
  public ObstacleAvoidance(Navigation nav) {
    this.nav = nav;
    safe = false;
  }

  /**
   * When run, the robot will follow the following routine:
   * (1) Turn clockwise until it detects the rightmost edge 
   * of the obstacle. 
   * (2) Turn past this edge by a small angle calculated to give 
   * the robot clearance to get past this edge.
   * (3) Move forward to where the edge was detected, and then
   * continue for a small distance meant to get the body of the robot past the
   * front-most face of the obstacle. 
   * (4) Return to the angle it was at when the routine began
   * (5) Move forward slightly, so that the center of rotation of the 
   * robot is just past the front-most face
   * (6) Turn 90deg counter clockwise, so that it (in theory)
   * is looking directly at the obstacle it just drove past.
   * (7) Repeat steps 1-3 once
   * (8) Return control to the navigation thread
   */
  public void run() {
    double startAngle = nav.getOdo().getXYT()[2];
    movePastEdge();
    nav.turnTo(startAngle);
    moveForward(15);
    nav.turnTo((startAngle + 270) % 360);
    movePastEdge();
    safe = true;
  }

  /**
   * Allows the robot to find the rightmostface of an obstacle,
   * and move to be alongside it.
   */
  public void movePastEdge() {
    double edgeDist = findEdge();
    double extraT = Math.toDegrees(Math.asin(Lab3.TRACK/2/edgeDist));
    nav.turnTo(nav.getOdo().getXYT()[2] + extraT, TURN_SPD);
    moveForward(edgeDist + EXTRA_DIST);
  }

  /**
   * Turns robot clockwise until the edge of the obstacle is detected,
   * then returns the distance of that edge.
   * @return The distance from the robot to the edge. -1 if no obstacle.
   */
  private double findEdge() {
    double dist = nav.readUS();
    double edgeDist = -1;

    nav.setSpeeds(TURN_SPD, TURN_SPD);

    Lab3.LEFT_MOTOR.forward();
    Lab3.RIGHT_MOTOR.backward();  
    while ((dist = nav.readUS()) < SAFE_DIST) {
      edgeDist = dist;
      try {
        sleep(30);
      } catch (Exception e) {}
    }
    return edgeDist;
  }

  /**
   * Moves the robot forward (straight) a certain distance,
   * using the odometer.
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

  /**
   * Checks whether or not the robot is still handling the emergency
   * @return True if it is done avoiding the obstacle, false otherwise
   */
  public boolean isResolved() {
    return safe;
  }
}
