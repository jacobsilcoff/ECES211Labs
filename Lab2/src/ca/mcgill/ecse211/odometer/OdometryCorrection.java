/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.lab2.Lab2;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
  private static final float LIGHT_THRESHOLD = 0.45f;
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
    long correctionStart, correctionEnd;

    while (true) {
      correctionStart = System.currentTimeMillis();

      // TODO Trigger correction (When do I have information to correct?)

      lightSensor.fetchSample(sample, 0);
      if (sample[0] < LIGHT_THRESHOLD) {
    	  // TODO Calculate new (accurate) robot position
    	  if (lastPos == null) {
    		  lastPos = odometer.getXYT();
    	  } else {
    		  //we have now reached a second line, ideally perpendicular to our trajectory!

              // TODO Update odometer with new calculated (and more accurate) values
              //Reasonably, can either correct angle or distance, but not necessarily both
    		  
    	  }
      }
      //If we've turned dramatically, this wont be useful, so nullify last data
      if (lastPos != null && Math.abs(odometer.getXYT()[3] - lastPos[3]) > T_THRESHOLD) {
    	  lastPos = null;
      }
     
      Lab2.lcd.drawString("LIGHT VAL: " + sample[0], 0, 5);


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
}
