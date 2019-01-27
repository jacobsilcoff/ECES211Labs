/*
 * OdometryCorrection.java
 * (0,0) is the leftmost gridline intersection. This means in the correction, robot will start at approx (-15,-15)
 * and the correction will compensate for this. 
 * Final displayed XYT should be with respect to the gridlines, unlike in uncorrected odometer, where start
 * and final positions should be (0,0)
 */
package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.lab2.Lab2;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
  private enum CorrectionType{HEADING, DISTANCE}
  private static final float DIST_THRESHOLD = 5; //cm
  
  private static final CorrectionType CORRECTION = CorrectionType.DISTANCE;
 
  private static final float LIGHT_THRESHOLD = 0.33f; //light threshold (detects black lines)
  private static final int T_THRESHOLD = 10; //theta threshold (degrees)
  private static final float LINE_SPACING = 30.48f;
  private static final long CORRECTION_PERIOD = 10;
 
  private static double OFFSET_X = 15.24; //ideal starting x-offset
  private static double OFFSET_Y = 15.24; //ideal starting y-offset
  
  
  private Odometer odometer;
  private SampleProvider lightSensor;
  private float[] sample; //store light sensor sample
  
  private double[] lastPos; //store XYT at last line
  
  
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
   * run method for odometer correction (required for Thread)
   * @throws OdometerExceptions
   */
  public void run() {
	int lineCount = 0;
    long correctionStart, correctionEnd;

    
    while (true) {
      correctionStart = System.currentTimeMillis();

      // TODO Trigger correction (When do I have information to correct?)
      lightSensor.fetchSample(sample, 0);
      
      //TODO: FILTER LIGHT SENSOR!!! MAKE SURE YOU"VE moved first
      //done by checking if light sensor sample is less than the black line detection threshold
      
     
      //To avoid a single line triggering this many times, verify that either
      //we haven't seen a line yet at all (lastPos == null) or we're sufficiently
      //far from the last line.
      if (sample[0] < LIGHT_THRESHOLD && 
    		  (lastPos == null || dist(lastPos, odometer.getXYT()) > DIST_THRESHOLD)) {
    	  
    	  lineCount++; //line detected
    	  Lab2.lcd.drawString(lineCount + " line(s) detected.", 0, 5);
    	  
    	  // TODO Calculate new (accurate) robot position
    	  if (lastPos == null) { //first line
    		  lastPos = odometer.getXYT();
    	  } else {
    		  //we have now reached a second line, ideally perpendicular to our trajectory!

              // TODO Update odometer with new calculated (and more accurate) values
    		  double[] pos = odometer.getXYT(); //current odo-position
    		 
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
    		  
    		  //distance correction (note: cannot correct the first line going in any direction because starts from centre
    		  if (CORRECTION == CorrectionType.DISTANCE) {
    			  
    			  //1) upward (+Y) path: theta=0, y>x
    			 if ((pos[2]<T_THRESHOLD || (360-pos[2]<T_THRESHOLD) )&& pos[1]>pos[0]){  //theta approximately 0deg	 
    				  odometer.setY(lastPos[1]+LINE_SPACING); //correct Y-value
    				  Lab2.lcd.drawString("CORRECTED UP.", 0, 6); //display check	
    			  }
    			  //2) rightward (+X) path: theta=90, y>x
    			  else if (Math.abs(pos[2]-90)<T_THRESHOLD && (pos[1]>pos[0]) && (pos[0]-lastPos[0]>LINE_SPACING-DIST_THRESHOLD)){ 
    				  odometer.setX(lastPos[0]+LINE_SPACING); //correct X-value
    				  Lab2.lcd.drawString("CORRECTED RIGHT.", 0, 6); //display check
    				 
    			  }
    			 //3) downward (-Y) path: theta = 180, x>y
    			  else if (Math.abs(pos[2]-180)<T_THRESHOLD && pos[0]>pos[1] && (lastPos[1]-pos[1]>LINE_SPACING-DIST_THRESHOLD)){  
    				  odometer.setY(lastPos[1]-LINE_SPACING); //correct Y-value
    				  Lab2.lcd.drawString("CORRECTED DOWN.", 0, 6); //display check
    			 }
    			  //4)leftward path theta = 270, x>y
    			  else if (Math.abs(pos[2]-270)<T_THRESHOLD && pos[0]>pos[1] && (lastPos[0]-pos[0]>LINE_SPACING-DIST_THRESHOLD)){
     				 	  odometer.setX(lastPos[0]-LINE_SPACING); //correct Y-value
      					  Lab2.lcd.drawString("CORRECTED LEFT.", 0, 6); //display check
       			 }
    			  
    			  //finished correcting odometer, set currrent position as lastPos
    			  lastPos = pos;
    			  
    			  //old correction code. commented out for now.
    			  //these multipliers should be elements of {-1,1,0}
    			//  int sin = (int) (Math.sin(Math.toRadians(pos[2]) + 0.5));
    			//  int cos = (int) (Math.cos(Math.toRadians(pos[2]) + 0.5));
    			//  if (sin == 0) {
    				  //y
    				//  odometer.setXYT(pos[0], lastPos[1] + sin * LINE_SPACING, pos[2]);
    			//  } else {
    			//	  //x change
    			//	  odometer.setXYT(lastPos[1] + cos * LINE_SPACING, pos[1], pos[2]);
    			 
    			  }
    		  }
    	  }
      
      //unneccessary in new code because checks error threshold
      //If we've turned dramatically, this wont be useful, so nullify last data
      //try {
    //	  if (lastPos != null && Math.abs(odometer.getXYT()[2] - lastPos[2]) > T_THRESHOLD) { //check this
    //		  lastPos = null;
   // 	  }
   //   } catch (NullPointerException npe) {//do nothing
   // 	  }
      
    //  Lab2.lcd.drawString(lineCount + " line(s) detected", 0, 5);
     
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
      
 
  }//end run method
  
  
  //calculates distance between two positions
  private static double dist(double[] a, double[] b) {
	  if (a.length < 2 || b.length < 2) {
		  return -1;
	  }
	  return Math.sqrt(Math.pow(a[0]-b[0], 2) + Math.pow(a[1]-b[1],2)); //fixed distance formula
  }
  
  
  //do we need to do this:
  //converts the XYT values to the gridline coordinate system  
  private double[] convertCoord(double[] pos){
	  double[] coord = pos;
	  
	  //change X
	  coord[0] = coord[0] - OFFSET_X; //change X wrt (0,0)
	  coord[1] = coord[1] - OFFSET_Y; //change Y wrt (0,0)
	  coord[2] = 270 - Math.asin(coord[1]/coord[0]); //change theta wrt (0,0)
	  return coord;
  }
   
}
