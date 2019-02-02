package ca.mcgill.ecse211.lab3;

public class ObstacleAvoidance extends Thread{
	
	private Navigation nav;
	private boolean safe;
	public static final int SAFE_DIST = 1000;
	public static final int TURN_SPD = 50;
	
	
	public ObstacleAvoidance(Navigation nav) {
		this.nav = nav;
		safe = false;
	}
	
	public void run() {
		//Obstacle avoidance code here
		double distNeeded = 0;
		/*
		 * Turns until it sees an optimal angle
		 * -- might need to turn a bit more to miss the block!
		 */
		nav.setSpeeds(TURN_SPD, -TURN_SPD);
		while ((distNeeded = nav.readUS()) < SAFE_DIST) {
			try {
				sleep(30);
			} catch (Exception e) {}
		}
		nav.setSpeeds(0, 0);
		//rotate an extra 10 degrees to keep things more or less safe
		nav.turnTo(nav.getOdo().getXYT()[2] + 10);
		//go forward an extra decimeter to avoid anything
		nav.moveForward(distNeeded + 10);
		safe = true;
	}

	public boolean isResolved() {
		return safe;
	}
}
