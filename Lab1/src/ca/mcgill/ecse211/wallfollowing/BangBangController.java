package ca.mcgill.ecse211.wallfollowing;

/*
 * Bang-Bang Controller - GROUP 71
 * 
 * Our bang-bang controller implements a constant forward speed different from the motorHigh
 * turning speed to allow for slower/cautious turning while maintaining faster forward motion
 * when within error threshold.
 * 
 * It also implements a corner factor to make tighter (faster) turns for 
 * concave corners and wider (slower) turns for convex.
 * This allows for the vehicle to make appropriate turning paths around corners while still
 * implementing the bang-bang controller method using a constant adjustment in speed
 * 
 * Filtering was also incorporated to account for sensor/hardware inaccuracies.
 */

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController{
 
  private static final int FORSPEED = 170; // Speed of robot in forward motion (when within error threshold) (deg/sec)
  private static final float CORNER_FACTOR = 0.25f; //reduction factor of right-hand turning speed for convex corners, increase speed for concave corners
  
  private static final int FILTER_OUT = 25;
  private int filterControl;
  
  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;
  

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    filterControl = 0;
    WallFollowingLab.leftMotor.setSpeed(0); // Start robot moving not moving
    WallFollowingLab.rightMotor.setSpeed(0);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
	  float cornerCorrection = (motorHigh - motorLow) * CORNER_FACTOR;
	  
	  //Adjusted filter for sensor data - filters out inconsistent readings that our hardware kept getting (50m+)
	  if (distance >= 1000 && filterControl < FILTER_OUT) {
	      // bad value, do not set the distance var, however do increment the
	      // filter value
	      filterControl++;
	    } else if (distance >= 20000) {
	    	//ignore these completely because 200m readings are inaccurate
	    } else if (distance >= 1000) {
	      // We have repeated large values, so there must actually be nothing
	      // there: leave the distance alone
	      this.distance = distance;
	    } else {
	      // distance went below 100: reset filter and leave
	      // distance alone.
	      filterControl = 0;
	      this.distance = distance;
	    }
	  
	// TODO: process a movement based on the us distance passed in (BANG-BANG style)
	    //ASSUME SENSOR ON LHS
	    if (Math.abs(bandCenter - this.distance) < bandwidth) {
	    	//Continue straight
	    	WallFollowingLab.leftMotor.setSpeed(FORSPEED);
	    	WallFollowingLab.rightMotor.setSpeed(FORSPEED);
	    	WallFollowingLab.leftMotor.forward();
	    	WallFollowingLab.rightMotor.forward();
	   
	    } else if (this.distance < bandCenter) {
	    	//go away from the wall using adjusted motorHigh speed (concave corner)
	    	WallFollowingLab.leftMotor.setSpeed(motorLow);
	    	WallFollowingLab.rightMotor.setSpeed(motorHigh + cornerCorrection);
	    	if (motorLow > 0)
	      		WallFollowingLab.leftMotor.forward();
	      	else 
	      		WallFollowingLab.leftMotor.backward(); //in-case of negative speeds
	    	WallFollowingLab.rightMotor.forward();
	    } else {
	    	//go towards the wall (convex corner)
	    	WallFollowingLab.leftMotor.setSpeed(motorHigh  - cornerCorrection);
	    	WallFollowingLab.rightMotor.setSpeed(motorLow);
	      	WallFollowingLab.leftMotor.forward();
	      	if (motorLow > 0)
	      		WallFollowingLab.rightMotor.forward();
	      	else 
	      		WallFollowingLab.rightMotor.backward(); //in-case of negative speeds
	    }
  }
  
  @Override
  public int readUSDistance() {
	    return this.distance;
  }
  
    
}