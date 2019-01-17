package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {
  private static final int FILTER_OUT = 20;
  private static final int FORSPEED = 100;
  private static final float CORNER_FACTOR = 0.5f;
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
    WallFollowingLab.leftMotor.backward();
    WallFollowingLab.rightMotor.backward();
  }

  @Override
  public void processUSData(int distance) {
	  if (distance >= 100 && filterControl < FILTER_OUT) {
	      // bad value, do not set the distance var, however do increment the
	      // filter value
	      filterControl++;
	    } else if (distance >= 20000) {//ignore these completely
	    	
	    }
	    else if (distance >= 100) {
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
    	
    	WallFollowingLab.leftMotor.backward();
    	WallFollowingLab.rightMotor.backward();
   
    } else if (this.distance > bandCenter) {
    	//go towards the wall
    	WallFollowingLab.leftMotor.setSpeed(motorLow);
    	WallFollowingLab.rightMotor.setSpeed(motorHigh / CORNER_FACTOR);
    	if (motorLow > 0)
      		WallFollowingLab.leftMotor.backward();
      	else 
      		WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.backward();
    } else {
    	//go away from the wall
    	WallFollowingLab.leftMotor.setSpeed(motorHigh);
    	WallFollowingLab.rightMotor.setSpeed(motorLow * CORNER_FACTOR);
      	WallFollowingLab.leftMotor.backward();
      	if (motorLow > 0)
      		WallFollowingLab.rightMotor.backward();
      	else 
      		WallFollowingLab.rightMotor.forward();
    }
    
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
