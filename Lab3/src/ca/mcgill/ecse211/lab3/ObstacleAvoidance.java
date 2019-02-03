package ca.mcgill.ecse211.lab3;

public class ObstacleAvoidance extends Thread{
	
	private Navigation nav;
	private boolean safe;
	private double distNeeded;
	private static final int SAFE_T = 10; //extra theta to turn to avoid obstacle (deg)
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
		
		
		
		//2.forward an extra decimeter to avoid anything
		//must check obstacle simultaneously
		int i =0;
		while (3*i< distNeeded +10) {
			if(nav.checkEmergency()) {
				turnAvoid();
				i = 0; //reset i if new obstacle detected
			}
			else {
				nav.moveForward(3); // move by 3cm each time
				i++;
			}
		}
		
		safe = true;
		
		
	}

	/**
	 * Turns robot until an optimal angle to avoid obstacle
	 */
	private void turnAvoid() {
		distNeeded = nav.readUS();				
		nav.setSpeeds(TURN_SPD, TURN_SPD);
		nav.leftMotor.forward();
		nav.rightMotor.backward();
			
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
	
	
	public boolean isResolved() {
		return safe;
	}
}
