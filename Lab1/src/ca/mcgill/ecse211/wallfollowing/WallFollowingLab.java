package ca.mcgill.ecse211.wallfollowing;
/*
 * Wall-Following Lab - GROUP 71 - Jacob Silcoff (260767897) and Helen Lin (260715521)
 * 
 * Parameters for band-center, bandwidth and turning (motor) speeds were adjusted for optimal performance.
 * 
 * Note that our bang-bang controller implements a constant forward speed different from the motorHigh
 * turning speed here, and that it also implements a corner factor to make tighter (faster) turns for
 * concave corners, and slower turns for convex (see bang-bang controller code)
 * 
 * Note that our p-controller implements an independent motorspeed indicated in its class
 * 
 */

import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;

public class WallFollowingLab {

  // Parameters: adjusted for desired performance

  private static final int bandCenter = 30; 	// Offset from the sensor to wall (note: sensor installed at 45deg) (cm)
  private static final int bandWidth = 1; 		// Width of dead band (cm)
  private static final int motorLow = 110; 		// Speed of slower rotating wheel (deg/sec)
  private static final int motorHigh = 310; 	// Speed of the faster rotating wheel (deg/sec)
  

  private static final Port usPort = LocalEV3.get().getPort("S1");
  public static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  
  public static int instructionNum = 0;

  // Main entry point - instantiate objects used and set up sensor

  public static void main(String[] args) {

    int option = 0;
    Printer.printMainMenu(); // Set up the display on the EV3 screen
    while (option == 0) // and wait for a button press. The button
      option = Button.waitForAnyPress(); // ID (option) determines what type of control to use

    // Setup controller objects

    BangBangController bangbangController =
        new BangBangController(bandCenter, bandWidth, motorLow, motorHigh);

    PController pController = new PController(bandCenter, bandWidth);

    // Setup ultrasonic sensor
    // There are 4 steps involved:
    // 1. Create a port object attached to a physical port (done already above)
    // 2. Create a sensor instance and attach to port
    // 3. Create a sample provider instance for the above and initialize operating mode
    // 4. Create a buffer for the sensor data

    @SuppressWarnings("resource") // Because we don't bother to close this resource
    SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
    SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
                                                              // this instance
    float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
                                                         // returned

    // Setup Printer
    // This thread prints status information in the background
    Printer printer = null;

    // Setup Ultrasonic Poller // This thread samples the US and invokes
    UltrasonicPoller usPoller = null; // the selected controller on each cycle

    // Depending on which button was pressed, invoke the US poller and printer with the
    // appropriate constructor.

    switch (option) {
      case Button.ID_LEFT: // Bang-bang control selected
        usPoller = new UltrasonicPoller(usDistance, usData, bangbangController);
        printer = new Printer(option, bangbangController);
        break;
      case Button.ID_RIGHT: // Proportional control selected
        usPoller = new UltrasonicPoller(usDistance, usData, pController);
        printer = new Printer(option, pController);
        break;
      default:
        System.out.println("Error - invalid button"); // None of the above - abort
        System.exit(-1);
        break;
    }

    // Start the poller and printer threads
    usPoller.start();
    printer.start();

    // Wait here forever until button pressed to terminate wallfollower
    Button.waitForAnyPress();
    System.exit(0);

  }
}