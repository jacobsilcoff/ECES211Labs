package ca.mcgill.ecse211.lab4;

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
public class Lab4 {

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

  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    //OdometryCorrection odometryCorrection = new OdometryCorrection();
    Display odometryDisplay = new Display();

    LCD.clear();
    // ask the user whether odometery correction should be run or not
    LCD.drawString("< Left | Right >", 0, 0);
    LCD.drawString(" Rising| Falling   ", 0, 1);
    LCD.drawString(" edge  | edge  ", 0, 2);
    LCD.drawString("       |         ", 0, 3);
    buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

    // Start odometer and display threads
    (new Thread(Odometer.getOdometer())).start();
    (new Thread(odometryDisplay)).start();
    
    UltrasonicLocalizer.Mode mode = UltrasonicLocalizer.Mode.FALLING_EDGE;
    if (buttonChoice == Button.ID_LEFT) {
      mode = UltrasonicLocalizer.Mode.RISING_EDGE;
    }
    //Begins ultrasonic correction and awaits completion
    UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(mode);
    usLocalizer.start();
    try {
      usLocalizer.join();
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    LCD.clear();
    LCD.drawString("Use any button", 0, 1);
    LCD.drawString("to begin light", 0, 1);
    LCD.drawString("localization.  ", 0, 2);
    Button.waitForAnyPress();
    LightLocalizer lightLoc = new LightLocalizer(); 
    lightLoc.start();
    
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
