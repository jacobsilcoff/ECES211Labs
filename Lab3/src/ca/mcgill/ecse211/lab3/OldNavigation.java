package ca.mcgill.ecse211.lab3;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/*
 * A class used to navigate the robot through the obstacles
 */
public class OldNavigation {
	
	private static final int FORWARD_SPEED = 250;//make 250 later
	private static final int ROTATE_SPEED = 150;
	private static final double TILE_SIZE = 30.48;
	private static final double DIST_THRESH = 1;
	
	private static final int DOWN_TIME = 300;//time to wait after doing a thing for testing
	
	private static final int SLEEP_TIME = 10;//10 ms sleep cycle
	
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private double lRad;
	private double rRad;
	private double track;
	private boolean isNavigating;
	private Odometer odo;
	private SampleProvider usSensor;
	
	private static int numInstructions = 0;
	
	public OldNavigation(EV3LargeRegulatedMotor left, EV3LargeRegulatedMotor right,
			Odometer odometer, double leftRadius, double rightRadius, double track, 
			SampleProvider usSensor) {
		leftMotor = left;
		rightMotor = right;
		odo = odometer;
		this.track = track;
		lRad = leftRadius;
		rRad = rightRadius;
		isNavigating = false;
	}
	
	public void travelToWaypoints(double[][] pts) {
		for (double[] pt : pts) {
			//ignore faulty points
			if (pt.length !=2)
				continue;
			travelTo(pt[0]*TILE_SIZE,pt[1]*TILE_SIZE, true);
		}
	}
	
	//For testing turnTo method
	public void turnToThetas(double[] ts) {
		for (double t : ts) {
			turnTo(t, true);
		}
	}
	
	public void travelTo(final double x, final double y) {
		travelToThread(x, y);
	}
	public void turnTo(double theta) {
		turnToThread(theta);
	}
	public void travelTo(final double x, final double y, boolean wait) {
		numInstructions++;
		ca.mcgill.ecse211.lab3.Lab3.lcd.drawString("Start ins " + numInstructions + "      ", 0, 4);
		Thread t = travelToThread(x,y);
		if (wait) {
			try {
				t.join();
				ca.mcgill.ecse211.lab3.Lab3.lcd.drawString("End ins " + numInstructions + "     ", 0, 4);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	public void turnTo(double theta, boolean wait) {
		numInstructions++;

		ca.mcgill.ecse211.lab3.Lab3.lcd.drawString("Start ins " + numInstructions + "      ", 0, 4);
		Thread t = turnToThread(theta);
		if (wait) {
			try {
				t.join();
				ca.mcgill.ecse211.lab3.Lab3.lcd.drawString("End ins " + numInstructions + "      ", 0, 4);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	public boolean isNavigating() {
		return isNavigating;
	}
	
	/**
	 * Travels to a point on an independant thread and returns said thread
	 * @param x the x coordinate
	 * @param y the y coordinate
	 * @return the thread used to get to that point
	 */
	private Thread travelToThread(final double x, final double y){		
		Thread t = new Thread(){
			public void run() {
				isNavigating = true;
				try { runHelper();}catch(InterruptedException e) {e.printStackTrace();}
				isNavigating = false;
			}
			public void runHelper() throws InterruptedException{		
				double dx = x - odo.getXYT()[0];
				double dy = y - odo.getXYT()[1];
				double dist = Math.sqrt((dx*dx) + (dy*dy));
				double theta;
				if (dy == 0) {
					theta = dx > 0 ? 90 : 270;
				} else {
					theta = Math.toDegrees(Math.atan(dx/dy)) 
							+ ((dy > 0)? 0 : 180);
					theta = (theta + 360) % 360; //normalize theta
				}
				//turn to angle
				turnTo(theta, true);
				
				isNavigating = true;
				//drive forward
				leftMotor.setSpeed(FORWARD_SPEED);
			    rightMotor.setSpeed(FORWARD_SPEED);
			    //Note: check against odometer!
			    leftMotor.rotate(convertDistance(lRad, dist), true);
			    rightMotor.rotate(convertDistance(rRad, dist), false);//change later...
			    sleep(DOWN_TIME);
//			    while (dist(odo.getXYT(), new double[] {x,y}) > DIST_THRESH ) {
//			    	
//			    	/*
//			    	 * Here we can do things like look out for obstacles,
//			    	 * or implement a p controller with the odometer
//			    	 */
//			    	try {
//						sleep(SLEEP_TIME);
//					} catch (InterruptedException e) {
//						e.printStackTrace();
//					}
//			    }
			}
		};
		t.start();
		return t;
	}
	
	//consider converting to a thread...
	private Thread turnToThread(final double theta) {
		Thread t = new Thread() {
			public void run() {
				isNavigating = true;
				try { runHelper();}catch(InterruptedException e) {
					ca.mcgill.ecse211.lab3.Lab3.lcd.drawString("ERROR", 0, 6);
				}
				isNavigating = false;
			}
			
			public void runHelper() throws InterruptedException{
				//Start by finding the rotational direction
				double presTheta = odo.getXYT()[2];
				double ang = (theta - presTheta + 360) % 360;
				leftMotor.setSpeed(ROTATE_SPEED);
			    rightMotor.setSpeed(ROTATE_SPEED);
				if (ang < 180) {
					ca.mcgill.ecse211.lab3.Lab3.lcd.drawString("Ang: " + ang + "deg  ", 0, 5);
					//increase angle
				    leftMotor.rotate(convertAngle(lRad, track, ang), true);
				    rightMotor.rotate(-convertAngle(rRad, track, ang), false);//make false for rough answer
				} else {
					ang = 360 - ang;
					ca.mcgill.ecse211.lab3.Lab3.lcd.drawString("Ang: " + ang+"deg   ", 0, 5);
				    //Need to check against odometer!
				    leftMotor.rotate(-convertAngle(lRad, track, ang), true);
				    rightMotor.rotate(convertAngle(rRad, track, ang), false);
				}
				sleep(DOWN_TIME);
			}
		};
		t.start();
		return t;
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
