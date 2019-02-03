package ca.mcgill.ecse211.lab3;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/*
 * A class used to navigate the robot according to waypoints and obstacles.
 */
public class Navigation extends Thread{
	
	private static final int FORWARD_SPEED = 250;		//forward speed (deg/s)
	private static final int ROTATE_SPEED = 150;		//turning speed (deg/s)
	private static final double TILE_SIZE = 30.48;		//grid spacing (cm)
	private static final double DIST_THRESH = 1;		//distance threshold (cm)
	private static final double EMERGENCY_THRESH = 10;	//emergency threshold for US sensor obstacle threshold distance (cm)
	private static final double T_THRESH = 0.5;			//turning angle threshold (deg)
	
	private static final int SLEEP_TIME = 30;			//sleep cycle (ms)
	
	public final EV3LargeRegulatedMotor leftMotor; //public to allow for obstacle avoidance class to turn motors
	public final EV3LargeRegulatedMotor rightMotor;
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
	
	/**
	 * default constructor for navigation (called in Lab3.java)
	 * @param left 		left motor
	 * @param right 	right motor
	 * @param odometer	odometer instance
	 * @param leftRadius	left wheel radius
	 * @param rightRadius	right wheel radius
	 * @param track			track / wheelbase 
	 * @param usSensor		US sensor sample provider
	 */
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
		destX = destY = destT = 0; //default destination is (0,0,0)
		this.usSensor = usSensor; //added
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
		ca.mcgill.ecse211.lab3.Lab3.lcd.drawString("Dest:" + (int)destX +"," +(int)destY +"," +(int)destT, 0, 4);
	
	}
	
	public Odometer getOdo() {
		return odo;
	}
	
	/**
	 * Moves the robot forward (straight) a certain distance. Used in obstacle avoidance
	 * @param dist
	 */
	public void moveForward(double dist) {
		leftMotor.forward();
		rightMotor.forward();
		
		this.setSpeeds(FORWARD_SPEED/4, FORWARD_SPEED/4); //move cautiously
	    //Note: check against odometer!
	    leftMotor.rotate(convertDistance(lRad, dist), true);
	    rightMotor.rotate(convertDistance(rRad, dist), false);
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
	 */
	public void turnTo(double theta) {
		double presTheta = odo.getXYT()[2]; //get current heading
		double ang = (theta - presTheta + 360) % 360; //gets absolute angle required to turn
		leftMotor.setSpeed(ROTATE_SPEED);
	    rightMotor.setSpeed(ROTATE_SPEED);
		
	    //turn using MINIMUM angle
	    if (ang < 180) {
			ca.mcgill.ecse211.lab3.Lab3.lcd.drawString("Ang: " + ang + "deg  ", 0, 5); //display angle of rotation
			//increase angle
		    leftMotor.rotate(convertAngle(lRad, track, ang), true);
		    rightMotor.rotate(-convertAngle(rRad, track, ang), false);//make false for rough answer
		} else {
			ang = 360 - ang; 
			ca.mcgill.ecse211.lab3.Lab3.lcd.drawString("Ang: " + ang+"deg   ", 0, 5); //display angle of rotation
		    //Need to check against odometer
		    leftMotor.rotate(-convertAngle(lRad, track, ang), true);
		    rightMotor.rotate(convertAngle(rRad, track, ang), false);
		}
	    updateT();//update new angle after turn;
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
				ca.mcgill.ecse211.lab3.Lab3.lcd.drawString("State: INIT", 0, 6); //display state
				if (isNavigating) {
					state = State.TURNING;
				}
				break;
			case TURNING:
				ca.mcgill.ecse211.lab3.Lab3.lcd.drawString("State: TURN", 0, 6);
				turnTo(destT); //turn robot to destination theta
				if (facing(destT)) { //if turned, start travel
					state = State.TRAVELING;
				}
				break;
			case TRAVELING:
				ca.mcgill.ecse211.lab3.Lab3.lcd.drawString("State: TRVL", 0, 6);
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
				ca.mcgill.ecse211.lab3.Lab3.lcd.drawString("State: EMRG", 0, 6);
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
		ca.mcgill.ecse211.lab3.Lab3.lcd.drawString("US:" +(usData[0] * 100.0), 0, 7);
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
		leftMotor.forward();
		rightMotor.forward();
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
	
	/**
	 * Gets distance (cm) between two coordinates
	 * @param a position array 1
	 * @param b position array 2
	 * @return distance (cm)
	 */
	private static double dist(double[] a, double[] b) {
		if (a.length < 2 || b.length < 2) {
			return -1;
		}
		return Math.sqrt(Math.pow(a[0]-b[0], 2) + Math.pow(a[1]-b[1],2)); //minimum distance formula
	}

}
