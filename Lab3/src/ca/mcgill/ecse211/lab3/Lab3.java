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

/**
 * This is the main class for Lab3. 
 * It allows the user to choose whether or not the robot is corrected,
 * and start it on a course of predefined waypoints.
 * @author Group 71, Helen Lin & Jacob Silcoff
 */
public class Lab3 {

  /**
   * The list of waypoints the robot travels between
   */
  public static final double[][] WAYPOINTS = {{0,2}};
  /**
   * The robot's left motor
   */
  public static final EV3LargeRegulatedMotor LEFT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  /**
   * The robot's right motor
   */
  public static final EV3LargeRegulatedMotor RIGHT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  /**
   * The robot's ultrasonic sensor
   */
  public static final SampleProvider US_SENSOR;
  /**
   * The robot's light sensor
   */
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
  /**
   * The LCD used to output during the robot's journey
   */
  public static final TextLCD LCD = LocalEV3.get().getTextLCD();
  /**
   * The radius of the robot's tires
   */
  public static final double WHEEL_RAD = 2.20;
  /**
   * The distance between the robot's two wheels
   * A larger value equates to greater turns
   */
  public static final double TRACK = 15.279;
  public static Navigation nav;
 

  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    Odometer odometer = Odometer.getOdometer(); 
    OdometryCorrection odometryCorrection = new OdometryCorrection();

    Display odometryDisplay = new Display(LCD);

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
        for (double[] pt : WAYPOINTS) {
          nav.travelTo(pt[0], pt[1]);
          LCD.drawString("Go to: (" + pt[0] + "," + pt[1], 0, 3); 
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
