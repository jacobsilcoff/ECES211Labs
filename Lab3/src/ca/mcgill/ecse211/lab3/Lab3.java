package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/*
 * Main class for Lab3
 * @author Group 71, Helen Lin & Jacob Silcoff
 */
public class Lab3 {

  // Motor Objects and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final Port lightSensorPort = LocalEV3.get().getPort("S3");
  private static final Port usPort  = LocalEV3.get().getPort("S1");
  public static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static Navigation nav;
  public static final double WHEEL_RAD = 2.20; 	//wheel radius (cm)
  public static final double TRACK = 15.279;		//wheel-base (cm) bigger = greater turns
  

  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
    

    @SuppressWarnings("resource") // Because we don't bother to close this resource
    SensorModes lightSensorMode = new EV3ColorSensor(lightSensorPort); // lightSensorMode is the instance
    SampleProvider lightSensor = lightSensorMode.getMode("Red");
    SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
    SampleProvider usDistance = usSensor.getMode("Distance");
   
    OdometryCorrection odometryCorrection = new OdometryCorrection(lightSensor); //odometryCorrection instance
    
    
    Display odometryDisplay = new Display(lcd); // No need to change from Lab 2
    
    nav = new Navigation(leftMotor, rightMotor, odometer, WHEEL_RAD, WHEEL_RAD, 
    		TRACK, usDistance);
    nav.start();


	
	  // clear the display
	  lcd.clear();
	
	  // ask the user whether the motors should drive in a square or float
	  lcd.drawString("Press any button", 0, 0);
	  lcd.drawString("to start!", 0, 1);
	
	  Button.waitForAnyPress(); // Record choice (left or right press)
	  lcd.clear();
      

      // ask the user whether odometery correction should be run or not
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("  No   | with   ", 0, 1);
      lcd.drawString(" corr- | corr-  ", 0, 2);
      lcd.drawString(" ection| ection ", 0, 3);
      lcd.drawString("       |        ", 0, 4);

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

      // Start odometer and display threads
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();

      // Start correction if right button was pressed
      if (buttonChoice == Button.ID_RIGHT) {
        Thread odoCorrectionThread = new Thread(odometryCorrection);
        odoCorrectionThread.start();
      }
      
      
      //Starts the robot
      (new Thread() {
        public void run() {
          double[][] waypoints = {{0,2},{2,2},{2,0},{0,0}}; //waypoints to use for Test Data
          for (double[] pt : waypoints) {
        	  nav.travelTo(pt[0], pt[1]);
        	  ca.mcgill.ecse211.lab3.Lab3.lcd.drawString("Go to: (" + pt[0] + "," +pt[1], 0, 3); //check
        	  while (nav.isNavigating()) {
        		  try {
        			  Thread.sleep(500);
        		  } catch (Exception e) {}
        	  }
          }

        }
      }).start();
    

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
