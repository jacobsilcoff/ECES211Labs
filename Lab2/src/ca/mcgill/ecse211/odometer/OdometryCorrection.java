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
 
  private static final float LIGHT_THRESHOLD = 0.03f; //light threshold for sig difference
  private static final float LINE_SPACING = 30.48f;
  private static final float DIST_THRESHOLD = 3;
  private static final long CORRECTION_PERIOD = 7;//from 10 to 7
  private static final double SENSOR_Y = 1.5; //distance btwn sensor and wheel axis in Y dir

  
  private Odometer odometer;
  private SampleProvider lightSensor;
  
  private int x;
  private int y;
  
  private float[] sample; //store light sensor sample
  
  private CircularArray samples;

  private double[] lastPos; //store XYT at last line

  
  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection(SampleProvider lightSensor) throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    
    x = -1;
    y = -1;
    
    this.lightSensor = lightSensor;
    sample = new float[lightSensor.sampleSize()];
    lastPos = null;
    samples = new CircularArray();
  }

  /**
   * run method for odometer correction (required for Thread)
   * @throws OdometerExceptions
   */
  @Override
public void run() {
	int lineCount = 0;
    long correctionStart, correctionEnd;

    
    while (true) {
      correctionStart = System.currentTimeMillis();

      lightSensor.fetchSample(sample, 0);

	  //Lab2.lcd.drawString("Light Avg: " + samples.getAvg(), 0, 6);
	  //Lab2.lcd.drawString("Diff: " + (samples.getAvg() - sample[0]), 0, 7);
	  double[] pos = odometer.getXYT(); //current odo-position
      //To avoid a single line triggering this many times, verify that either
      //we haven't seen a line yet at all (lastPos == null) or we're sufficiently
      //far from the last line.
	 if (sample[0] < samples.avg - LIGHT_THRESHOLD  && (lastPos == null 
			|| dist(pos, lastPos) > DIST_THRESHOLD)) {
    	  
    	//Indicate detection of a line
    	lineCount++; 
    	Lab2.lcd.drawString(lineCount + " line(s) detected.", 0, 5);
    		 
    	int sin = (int) Math.round(Math.sin(Math.toRadians(pos[2])));
    	int cos = (int) Math.round(Math.cos(Math.toRadians(pos[2])));
    	//are we going + or -? This matters b/c x and y refer to grid squares, 
    	//not gridlines, so an x of 0 could mean we're crossing the line x=1 facing
    	//down, or x = 0 facing up. Thus, an offset is needed based on the direction
    	//Also note the SENSOR_Y term to indicate the center is the center of the wheel base
    	int dirOffset = (sin + cos > 0) ? 0:1; 
    	if (sin == 0) {
    		y += cos; 
    		odometer.setXYT(pos[0], LINE_SPACING * (y + dirOffset) - cos*SENSOR_Y, pos[2]);
    	} else if (cos == 0) { //implicit, but good to write out
    		x += sin;
    		odometer.setXYT(LINE_SPACING * (x + dirOffset) - sin*SENSOR_Y, pos[1], pos[2]);
    	}
    	

    	Lab2.lcd.drawString("(" + x + ", " + y + ")         ", 0, 6);
    	
       	//update last pos of line detected
    	lastPos = odometer.getXYT();  
    	
      }
	 
	 //Add the sample to the rolling average
      samples.add(sample[0]);
      
	 // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
      
    }//end loop
  }//end run method
  
  
  //calculates distance between two positions
  private static double dist(double[] a, double[] b) {
	  if (a.length < 2 || b.length < 2) {
		  return -1;
	  }
	  return Math.sqrt(Math.pow(a[0]-b[0], 2) + Math.pow(a[1]-b[1],2)); //fixed distance formula
  }
  
  /*
   * Stores a circular array of light values.
   * Buffered to keep a rolling average.
   * 
   * We assume N is small enough that populating the buffer
   * with data takes a trivial amount of time, although
   * making the average work for less than N samples would be
   * trivial with a conditional and a for loop.
   */
  private class CircularArray{
	  private static final int N = 5;
	  private float[] samples; 
	  private int sampleIndex;
	  private float avg;
	  
	  public CircularArray() {
		  samples = new float[N];
		  sampleIndex = 0;
		  avg = 0;
	  }
	  
	  public void add(float x) {
		  avg = avg + 1f/N * (x - samples[sampleIndex]);
		  samples[sampleIndex] = x;
		  sampleIndex = (sampleIndex + 1) % N;
	  }
  }
    
}
