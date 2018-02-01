package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This is the P Controller
 * @author Lily Li, Rene Gagnon
 *
 */
public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 160;
  private static final int FILTER_OUT = 60;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;
  
  private int difference;
  private static final double propConst = 4.0;
  private static final int maxCorrection = 50;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  /**
   * This method overrides the processUSData in the UltrasonicController
   * control the robot based on the distance the ultrasonic sensor detects 
   * @param distance
   */
  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }

    // TODO: process a movement based on the us distance passed in (P style)
    difference = this.distance - bandCenter;
    int diff;
    //case 1: the robot is within the range of the bandWidth
    if(Math.abs(difference) <= bandWidth) {
    		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
        WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
    }
    //case 2 : too close to wall
    else if(difference < 0) {
    		//case 1: too close to the wall
    		if(this.distance <= 10) {
    			WallFollowingLab.leftMotor.stop();
    			WallFollowingLab.rightMotor.stop();
    			WallFollowingLab.leftMotor.setSpeed(100);
    			WallFollowingLab.rightMotor.setSpeed(150);
    			WallFollowingLab.leftMotor.forward();
    			WallFollowingLab.rightMotor.backward();
    		}
    		//case 2: not yet hit the wall, adjust to right
    		else {
    			diff = calcProp(difference);
    			WallFollowingLab.leftMotor.setSpeed((int)(MOTOR_SPEED + 1.5*diff)); // Initalize motor rolling forward
    			WallFollowingLab.rightMotor.setSpeed((int)(0 /*MOTOR_SPEED - 0.7*diff*/));
            WallFollowingLab.leftMotor.forward();
            WallFollowingLab.rightMotor.backward();
    		}
    }
    //case 3: too far from wall
    else if(difference > 0) {
    		diff = calcProp(difference);
    		WallFollowingLab.leftMotor.setSpeed((int)(MOTOR_SPEED - 0.45*diff)); // Initalize motor rolling forward
    	    WallFollowingLab.rightMotor.setSpeed((int)(MOTOR_SPEED + diff));
    	    WallFollowingLab.leftMotor.forward();
    	    WallFollowingLab.rightMotor.forward();
    }
  }
  /**
   * this method returns a correction value based on the distance from the wall
   * @param diff is the difference between the distance and bandCenter
   * @return correction
   */

  public int calcProp(int diff) {
	  int correction;
	  diff = Math.abs(diff);
	  correction = (int)(propConst * (double) diff);
	  if(correction >= MOTOR_SPEED) {
		  correction = maxCorrection;
	  }
	  return correction;
  }
  
  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
