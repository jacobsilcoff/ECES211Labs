package ca.mcgill.ecse211.wallfollowing;

/**
 * p-Controller code - GROUP 71
 */
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 150; //forward speed
  private static final int FILTER_OUT = 7;
  

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(0); // Start robot not moving
    WallFollowingLab.rightMotor.setSpeed(0);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
	 
    // rudimentary filter - toss out invalid samples corresponding to null
    // signal (adjusted according to our test data)
    if (distance >= 500 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 20000) {
    	//ignore extremely high values (50m+ completely)
    }
    else if (distance >= 500) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 100: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }

    // TODO: process a movement based on the us distance passed in (P style)
    //need differential to be proportional to distance
    if (Math.abs(this.distance - bandCenter) < bandWidth) {
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); //continue straight
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    } else if (this.distance < bandCenter) {
    	//need to move away from the wall
    	
    	int p = (int) Math.min(10*Math.abs(this.distance - bandCenter), MOTOR_SPEED * 2); //p-constant for left-handed turns
    	if (p > MOTOR_SPEED) {
        	WallFollowingLab.leftMotor.setSpeed(p - MOTOR_SPEED);
        	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + p );
        	WallFollowingLab.leftMotor.backward();
        	WallFollowingLab.rightMotor.forward();
    	} else {
        	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - p);
        	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + p );
        	WallFollowingLab.leftMotor.forward();
        	WallFollowingLab.rightMotor.forward();
    	}

    } else {
    	//need to move toward from wall
    	int p = (int) Math.min(3*Math.abs(this.distance - bandCenter), 0.35*MOTOR_SPEED); //p-constant for right-handed turns
    	
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + p);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - p  );
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
    
  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}