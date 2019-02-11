package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

public class LightLocalizer extends Thread {

  private Odometer odo;
  private Navigation nav;

  /**
   * default constructor starts a navigation thread for light localizer
   */
  public LightLocalizer() throws OdometerExceptions{
    try {
      odo = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
    this.nav = new Navigation(); //start a new nav thread
    nav.start();

  }

  /**
   * Move robot to (0,0) position using nav system
   */
  private void movetoOrigin(){
    nav.travelTo(0,0);
    while (nav.isNavigating()) {
      try {
        Thread.sleep(500);
      } catch (Exception e) {
      }
    }
    
  }
  
  public void run() {
    this.movetoOrigin();
    
    //TODO: correct/localize lines at origin
  }
}
