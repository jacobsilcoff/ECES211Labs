package ca.mcgill.ecse211.lab3;

/**
 * This class represents a routine that can be used
 * by the robot to avoid hitting an obstacle. 
 * @author Team 71
 */
public class ObstacleAvoidance extends Thread{
	
	private Navigation nav;
	private boolean safe;
	private double distNeeded;
	private static final int SAFE_T = 35; //extra theta to turn to avoid obstacle (deg)
	public static final int SAFE_DIST = 50; //safe distance from obstacle (cm)
	public static final int TURN_SPD = 50;
	
	
	public ObstacleAvoidance(Navigation nav) {
		this.nav = nav;
		distNeeded = 0;
		safe = false;
	}
	
	public void run() {
		//Obstacle avoidance algorithm:
		
		//1. turn until an optimal avoidance angle is acheived
		turnAvoid();
		
		//2. move forward an extra decimeter to avoid anything. 
		int i =0;
		while (3*i< distNeeded +10) {
			if(nav.checkEmergency()) { // must check obstacle simultaneously
				turnAvoid();
				i = 0; //reset i if new obstacle detected
			}
			else {
				nav.moveForward(3); // move by 3cm each time
				i++;
			}
		}
		
		//3. move back around obstacle to double check twice
		for (int j=0; j<2;j++){
			turnBack();
			i = 0;
			while (3*i< distNeeded +5) {
				if(nav.checkEmergency()) {
					turnAvoid();
					i = 0; //reset i if new obstacle detected
				}
				else {
					nav.moveForward(3); // move by 3cm each time
					i++;
				}
			}
		}
		
		
		//two clearances made, should be safe
		safe = true;
		
		
	}

	/**
	 * Turns robot until an optimal angle to avoid obstacle
	 */
	private void turnAvoid() {
		distNeeded = nav.readUS();				
		nav.setSpeeds(TURN_SPD, TURN_SPD);
		Lab3.LEFT_MOTOR.forward();
		Lab3.RIGHT_MOTOR.backward();
			
		while (( nav.readUS()) < SAFE_DIST) { //turn until obstacle is not detected
			try {
				sleep(30);
			} catch (Exception e) {}
		}
		
		nav.setSpeeds(0, 0); //stop
		
		//rotate an extra angle to keep things more or less safe
		nav.turnTo(nav.getOdo().getXYT()[2] + SAFE_T);
		nav.setSpeeds(0, 0); //stop again
	}
	
	/**
	 * turn back method -- checks if obstacle is truly cleared
	 * @return
	 */
	private void turnBack() {
		nav.setSpeeds(0, 0); //stop
		
		//turn to halfway between current heading and dest heading and recheck obstacle
		double halfAng = ((nav.getDestT()+ nav.getOdo().getXYT()[2])/2 +360 )%360;
		nav.turnTo(halfAng);
		nav.setSpeeds(0, 0); //stop again
	}
	
	public boolean isResolved() {
		return safe;
	}
}
