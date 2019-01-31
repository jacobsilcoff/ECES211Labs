package ca.mcgill.ecse211.lab3;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/*
 * A class used to navigate the robot through the obstacles
 */
public class Navigation {
	
	private static final int FORWARD_SPEED = 250;//make 250 later
	private static final int ROTATE_SPEED = 150;
	private static final double TILE_SIZE = 30.48;
	private static final double DIST_THRESH = 1;
	
	private static final int SLEEP_TIME = 10;//10 ms sleep cycle
	
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private double lRad;
	private double rRad;
	private double track;
	private boolean isNavigating;
	private Odometer odo;
	
	public Navigation(EV3LargeRegulatedMotor left, EV3LargeRegulatedMotor right,
			Odometer odometer, double leftRadius, double rightRadius, double track) {
		leftMotor = left;
		rightMotor = right;
		odo = odometer;
		this.track = track;
		lRad = leftRadius;
		rRad = rightRadius;
		isNavigating = false;
	}
	
	public void travelTo(final double x, final double y) {
		
		isNavigating = true;
		(new Thread(){
			public void run() {
				double dx = x - odo.getXYT()[0];
				double dy = y - odo.getXYT()[1];
				double dist = Math.sqrt(x * x + y * y);
				double theta;
				if (dx == 0) {
					theta = dy > 0 ? 0 : 180;
				} else {
					theta = Math.toDegrees(Math.atan(dx/dy)) 
							+ ((dy > 0)? 0 : 180);
					theta = (theta + 360) % 360; //normalize theta
				}
				//turn to angle
				turnTo(theta);
				while (isNavigating) {
					try {
						sleep(SLEEP_TIME);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
				isNavigating = true;
				//drive forward
				leftMotor.setSpeed(FORWARD_SPEED);
			    rightMotor.setSpeed(FORWARD_SPEED);
			    //Note: check against odometer!
			    leftMotor.rotate(convertDistance(lRad, dist), true);
			    rightMotor.rotate(convertDistance(rRad, dist), true);
			    while (dist(odo.getXYT(), new double[] {x,y}) > DIST_THRESH ) {
			    	
			    	/*
			    	 * Here we can do things like look out for obstacles,
			    	 * or implement a p controller with the odometer
			    	 */
			    	try {
						sleep(SLEEP_TIME);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
			    }
			}
		}).start();
		isNavigating = false;
	}
	
	//consider converting to a thread...
	public void turnTo(final double theta) {
		isNavigating = true;
		(new Thread() {
			public void run() {
				//Start by finding the rotational direction
				double presTheta = odo.getXYT()[2];
				double ang = (theta - presTheta + 360) % 360;
				leftMotor.setSpeed(ROTATE_SPEED);
			    rightMotor.setSpeed(ROTATE_SPEED);
				if (ang < 180) {
					//increase angle
				    leftMotor.rotate(convertAngle(lRad, track, ang), true);
				    rightMotor.rotate(-convertAngle(rRad, track, ang), false);//make false for rough answer
				} else {
					ang -= 180;
					
				    //Need to check against odometer!
				    leftMotor.rotate(-convertAngle(lRad, track, 90.0), true);
				    rightMotor.rotate(convertAngle(rRad, track, 90.0), false);
				}
				isNavigating = false;
			}
		}).start();
		
	}
	
	public boolean isNavigating() {
		return isNavigating;
	}
	
	private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	  }

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	private static double dist(double[] a, double[] b) {
		if (a.length < 2 || b.length < 2) {
			return -1;
		}
		return Math.sqrt(Math.pow(a[0]-b[0], 2) + Math.pow(a[1]-b[1],2)); //fixed distance formula
	}

}
