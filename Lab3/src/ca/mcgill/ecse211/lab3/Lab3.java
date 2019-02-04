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
  public static final EV3LargeRegulatedMotor LEFT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  public static final EV3LargeRegulatedMotor RIGHT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  public static final SampleProvider US_SENSOR;
  public static final SampleProvider LIGHT_SENSOR;
  static {
    Port lightSensorPort = LocalEV3.get().getPort("S3");
    Port usPort  = LocalEV3.get().getPort("S1");

    @SuppressWarnings("resource")
    SensorModes lightSensorMode = new EV3ColorSensor(lightSensorPort); 
    LIGHT_SENSOR = lightSensorMode.getMode("Red");
    SensorModes usSensor = new EV3UltrasonicSensor(usPort);
    US_SENSOR = usSensor.getMode("Distance");

  }
  public static final TextLCD LCD = LocalEV3.get().getTextLCD();
  public static Navigation nav;
  public static final double WHEEL_RAD = 2.20; 	//wheel radius (cm)
  public static final double TRACK = 15.279;		//wheel-base (cm) bigger = greater turns


  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(); 
    OdometryCorrection odometryCorrection = new OdometryCorrection(); //odometryCorrection instance


    Display odometryDisplay = new Display(LCD); // No need to change from Lab 2

    nav = new Navigation(odometer);
    nav.start();



    // clear the display
    LCD.clear();

    // ask the user whether the motors should drive in a square or float
    LCD.drawString("Press any button", 0, 0);
    LCD.drawString("to start!", 0, 1);

    Button.waitForAnyPress(); // Record choice (left or right press)
    LCD.clear();


    // ask the user whether odometery correction should be run or not
    LCD.drawString("< Left | Right >", 0, 0);
    LCD.drawString("  No   | with   ", 0, 1);
    LCD.drawString(" corr- | corr-  ", 0, 2);
    LCD.drawString(" ection| ection ", 0, 3);
    LCD.drawString("       |        ", 0, 4);

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
        double[][] waypoints = {{1,0}, {2,1}, {2,2}, {0,2}, {1,1}}; //waypoints to use for Test Data
        for (double[] pt : waypoints) {
          nav.travelTo(pt[0], pt[1]);
          LCD.drawString("Go to: (" + pt[0] + "," +pt[1], 0, 3); //check
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
