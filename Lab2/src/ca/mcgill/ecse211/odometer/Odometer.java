/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  //why is it called track what a bad variable name...
  private final double TRACK;
  private final double WHEEL_RAD;
  //Multiply by degrees to get distance moved by a single wheel
  private final double DIST_MULT;

  private double[] position;


  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms, equiv to 40Hz

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    //reset tacho count in motor
    this.leftMotor.resetTachoCount();
    this.rightMotor.resetTachoCount();

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);
    
    //reset tacho count in motor
    this.leftMotor.resetTachoCount();
    this.rightMotor.resetTachoCount();

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;
    this.DIST_MULT = (Math.PI * WHEEL_RAD / 180);

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();
      
      //Measure differences then update
      int leftDiff = leftMotor.getTachoCount() - leftMotorTachoCount;
      int rightDiff = rightMotor.getTachoCount() - rightMotorTachoCount;
      
      leftMotorTachoCount += leftDiff;
      rightMotorTachoCount += rightDiff;
      
      
      // TODO Calculate new robot position based on tachometer counts
      
      double leftDist = leftDiff * DIST_MULT; //left wheel distance traveled
      double rightDist = rightDiff * DIST_MULT; //right wheel distance traveled
      double disp = 0.5*(leftDist + rightDist); //vehicle displacement in the forward direction (average)
     
      double dx, dy, dt; //displacement components in the x, y, and theta direction (heading
      
    
      dt = Math.toDegrees((leftDist-rightDist)/TRACK); //No arcsin needed b/c small angle
      
      //idk if adding full weight of dt is wise... maybe half of it? shouldnt matter much
      dx = disp * Math.sin(Math.toRadians(odo.getXYT()[2] + dt));
      dy = disp * Math.cos(Math.toRadians(odo.getXYT()[2] + dt));
      
      
      // TODO Update odometer values with new calculated values
      odo.update(dx, dy, dt);

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

}
