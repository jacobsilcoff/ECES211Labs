package ca.mcgill.ecse211.lab3;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/*
 * A class used to navigate the robot through the obstacles
 */
public class Navigation extends Thread{
	
	private static final int FORWARD_SPEED = 250;//make 250 later
	private static final int ROTATE_SPEED = 150;
	private static final double TILE_SIZE = 30.48;
	private static final double DIST_THRESH = 1;
	private static final double EMERGENCY_THRESH = 10;
	private static final double T_THRESH = 0.1;
	
	private static final int SLEEP_TIME = 30;//10 ms sleep cycle
	
	private final EV3LargeRegulatedMotor leftMotor;
	private final EV3LargeRegulatedMotor rightMotor;
	private final double lRad;
	private final double rRad;
	private final double track;
	private boolean isNavigating;
	private Odometer odo;
	private SampleProvider usSensor;
	
	private static int numInstructions = 0;
	private double destX;
	private double destY;
	private double destT;
	
	public Navigation(EV3LargeRegulatedMotor left, EV3LargeRegulatedMotor right,
			Odometer odometer, final double leftRadius, final double rightRadius, final double track, 
			SampleProvider usSensor) {
		leftMotor = left;
		rightMotor = right;
		odo = odometer;
		this.track = track;
		lRad = leftRadius;
		rRad = rightRadius;
		isNavigating = false;
		destX = destY = destT = 0;
	}
	
	/**
	 * Sets robot to travel to a given point,
	 * updating the destination direction and position
	 * @param x The desired x in cm
	 * @param y The desired y in cm
	 */
	public void travelTo(double x, double y) {
		destX = x;
		destY = y;
		updateT();
	}
	
	public Odometer getOdo() {
		return odo;
	}
	
	/**
	 * Moves the robot forward a certain distance.
	 * @param dist
	 */
	public void moveForward(double dist) {
		leftMotor.setSpeed(FORWARD_SPEED);
	    rightMotor.setSpeed(FORWARD_SPEED);
	    //Note: check against odometer!
	    leftMotor.rotate(convertDistance(lRad, dist), true);
	    rightMotor.rotate(convertDistance(rRad, dist), false);
	}
	
	public boolean isTraveling() {
		return isNavigating;
	}
	
	
	/**
	 * Turns the robot to a given angle theta
	 * @param theta The desired angle in degrees
	 */
	public void turnTo(double theta) {
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
	}
	
	enum State{INIT,TURNING,TRAVELING, EMERGENCY}
	
	/**
	 * Implements a state machine of initializing, turning
	 * taveling, or handling an emergency obstacle
	 */
	public void run() {
		State state = State.INIT;
		ObstacleAvoidance avoidance = new ObstacleAvoidance(this);
		while (true) {
			switch(state) {
			case INIT:
				if (isNavigating) {
					state = State.TURNING;
				}
				break;
			case TURNING:
				turnTo(destT);
				if (facing(destT)) {
					state = State.TRAVELING;
				}
				break;
			case TRAVELING:
				//make sure desired distance is correct
				updateT();
				if (checkEmergency()) {
					state = State.EMERGENCY;
					avoidance = new ObstacleAvoidance(this);
					avoidance.start();
				}
				else if (!facing(destT)) {
					state = State.TURNING;
				}
				else if (!checkIfDone()) {
					updateTravel();
				} else { //Arrived
					setSpeeds(0,0);
					isNavigating = false;
					state = State.INIT;
				}
				break;
			case EMERGENCY:
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
		float[] usData = new float[usSensor.sampleSize()];
		usSensor.fetchSample(usData, 0);
		return (int) (usData[0] * 100.0);
	}
	
	/*
	 * Sets the motor speeds proportional to the robots direction
	 */
	private void updateTravel() {
		double dist = dist(new double[] {destX,destY}, odo.getXYT());
		//slows down upon nearing destination
		if (dist > 20) {
			setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
		} else {
			float motorSpeeds = (float)(dist/20 * (FORWARD_SPEED - 50) + 50);
			setSpeeds(motorSpeeds, motorSpeeds);
		}
		leftMotor.forward();
		rightMotor.forward();
	}
	
	/**
	 * Updates the destT to reflect
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
	 * @return
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
		leftMotor.setSpeed(l);
		rightMotor.setSpeed(r);
	}
	private void setSpeeds(int l, int r) {
		leftMotor.setSpeed(l);
		rightMotor.setSpeed(r);
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
