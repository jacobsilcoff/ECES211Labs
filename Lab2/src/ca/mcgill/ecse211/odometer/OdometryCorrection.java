/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.lab2.Lab2;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
  private enum CorrectionType{HEADING, DISTANCE}
  private static final float DIST_THRESHOLD = 5;
  private static final CorrectionType CORRECTION = CorrectionType.DISTANCE;
  private static final float LIGHT_THRESHOLD = 0.49f;
  private static final int T_THRESHOLD = 15;
  private static final float LINE_SPACING = 30.48f;
  private static final long CORRECTION_PERIOD = 10;
  
  
  private Odometer odometer;
  private SampleProvider lightSensor;
  private float[] sample;
  
  private double[] lastPos;
  
  
  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection(SampleProvider lightSensor) throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();

    this.lightSensor = lightSensor;
    sample = new float[lightSensor.sampleSize()];
    lastPos = null;
  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
	int lineCount = 0;
    long correctionStart, correctionEnd;

    while (true) {
      correctionStart = System.currentTimeMillis();

      // TODO Trigger correction (When do I have information to correct?)
      
      //TODO: FILTER LIGHT SENSOR!!! MAKE SURE YOU"VE moved first
      
      
      lightSensor.fetchSample(sample, 0);
     
      //To avoid a single line triggering this many times, verify that either
      //we haven't seen a line yet at all (lastPos == null) or we're sufficiently
      //far from the last line.
      if (sample[0] < LIGHT_THRESHOLD && 
    		  (lastPos == null || dist(lastPos, odometer.getXYT()) > DIST_THRESHOLD)) {
    	  lineCount++;
    	 
    	  // TODO Calculate new (accurate) robot position
    	  if (lastPos == null) {
    		  lastPos = odometer.getXYT();
    	  } else {
    		  //we have now reached a second line, ideally perpendicular to our trajectory!

              // TODO Update odometer with new calculated (and more accurate) values
    		  double[] pos = odometer.getXYT();
    		 
    		  //heading correction
    		  if (CORRECTION == CorrectionType.HEADING) {
    			  //Here, we assume that the distance readings are accurate, and 
    			  //update the heading to match the read distance.
    			  
    			  // The issue is that although we can find an error, we don't know the sign
    			  //Haven't figured out a solution, but it's still cool to be able to 
    			  //Find the error in angle
    			  
    			  double predictedDistance = 
    					  Math.sqrt(Math.pow(lastPos[0] - pos[0], 2) +
    							  	Math.pow(lastPos[1] - pos[1], 2));
    			  //this should be true! If it isn't true, there is necessarily an error
    			  //in the distance reading, or we crossed a non-perpendicular line.
    			  if (predictedDistance > LINE_SPACING) {
    				  double tError = Math.toDegrees(Math.acos(predictedDistance/LINE_SPACING));
    				  //if we have some way of knowing the tendancy of the robot, we can choose to
    				  //add or subtract this error, but that will take testing, and will be imperfect.
    			  } else {
    				  //?????
    			  }
    			  
    			  
    		  } 
    		  
    		  //distance correction
    		  else if (CORRECTION == CorrectionType.DISTANCE) {
    			  //Here, we assume the angle reading was sufficient, and scale distance
    			  
    			  //these multipliers should be elements of {-1,1,0}
    			  int sin = (int) (Math.sin(pos[2]) + 0.5);
    			  int cos = (int) (Math.cos(pos[2]) + 0.5);
    			  if (sin == 0) {
    				  //y motion
    				  odometer.setXYT(pos[0], lastPos[1] + sin * LINE_SPACING, pos[2]);
    			  } else {
    				  //x change
    				  odometer.setXYT(lastPos[1] + cos * LINE_SPACING, pos[1], pos[2]);
    			  }
    		  }
    	  }
      }
      //If we've turned dramatically, this wont be useful, so nullify last data
      try {
    	  if (lastPos != null && Math.abs(odometer.getXYT()[2] - lastPos[2]) > T_THRESHOLD) {
    		  lastPos = null;
    	  }
      } catch (NullPointerException npe) {}
     
      Lab2.lcd.drawString(lineCount + " line(s) detected", 0, 5);


      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
  
  //calculates if current distance is within distance error threshold
  private static double dist(double[] a, double[] b) {
	  if (a.length < 2 || b.length < 2) {
		  return -1;
	  }
	  return Math.sqrt(Math.pow(a[0]-b[0], 2) + Math.pow(a[1]-b[1],2)); //fixed distance formula
  }
}
