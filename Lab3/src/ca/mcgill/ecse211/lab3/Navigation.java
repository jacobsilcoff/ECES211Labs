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
	
	public void travelTo(double x, double y) {
		
	}
	
	//consider converting to a thread...
	public void turnTo(double theta) {
		isNavigating = true;
		//Start by finding the rotational direction
		double presTheta = odo.getXYT()[2];
		double ang = (theta - presTheta + 360) % 360;
		if (ang < 180) {
			//increase angle
			leftMotor.setSpeed(ROTATE_SPEED);
		    rightMotor.setSpeed(ROTATE_SPEED);
		    leftMotor.rotate(convertAngle(lRad, track, ang), true);
		    rightMotor.rotate(-convertAngle(rRad, track, ang), false);
		} else {
			ang -= 180;
			leftMotor.setSpeed(ROTATE_SPEED);
		    rightMotor.setSpeed(ROTATE_SPEED);

		    leftMotor.rotate(-convertAngle(lRad, track, 90.0), true);
		    rightMotor.rotate(convertAngle(rRad, track, 90.0), false);
		}
		isNavigating = false;
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

}
