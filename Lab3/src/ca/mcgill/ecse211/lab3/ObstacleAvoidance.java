package ca.mcgill.ecse211.lab3;

public class ObstacleAvoidance extends Thread{
	
	private NavThread nav;
	private boolean safe;
	public ObstacleAvoidance(NavThread nav) {
		this.nav = nav;
		safe = false;
	}
	
	public void run() {
		//Obstacle avoidance code here
		
		safe = true;
	}

	public boolean isResolved() {
		return safe;
	}
}
