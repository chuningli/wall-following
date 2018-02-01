package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;
/**
 * This is the BangBang Controller
 * @author Lily Li, Rene Gagnon
 *
 */

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;
  //variables
  private int difference; //difference between distance and bandCenter
  private int filterNum;
  

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
    filterNum = 0;
  }

  /**
   * This method overrides the processUSData in the UltrasonicController
   * control the robot based on the distance the ultrasonic sensor detects 
   * @param distance
   */
  @Override
  public void processUSData(int distance) {
    //this.distance = distance;
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
    //filter to avoid gap
	if (distance >= 200 && filterNum < 40) {
	// bad value, do not set the distance var, however do increment the
	// filter value
		filterNum++;
	} 
	else if (distance >= 200) {
	// We have repeated large values, so there must actually be nothing
	// there: leave the distance alone
		this.distance = distance;
	} 
	else {
	// distance went below 255: reset filter and leave
	// distance alone.
		filterNum = 0;
		this.distance = distance;
	}
	  
    difference = this.distance - bandCenter;
    
    //case 1: within the range of bandwidth
    if(Math.abs(difference) < bandwidth) {
    		WallFollowingLab.leftMotor.setSpeed(motorHigh); 
    	    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    	    WallFollowingLab.leftMotor.forward();
    	    WallFollowingLab.rightMotor.forward();
    }
    
    //case 2: the robot is closer to the wall
    else if(difference < 0 - bandwidth){
    		//case 1: too close to the wall
    		if(this.distance <= 10) {
    			WallFollowingLab.leftMotor.stop();
	    		WallFollowingLab.rightMotor.stop();
    			WallFollowingLab.leftMotor.setSpeed(motorLow + 50);
    			WallFollowingLab.rightMotor.setSpeed(motorLow);
    			WallFollowingLab.leftMotor.forward();
	    		WallFollowingLab.rightMotor.backward();
    		}
    		//case 2: not yet hit the wall, but too close
    		else{
    			WallFollowingLab.leftMotor.setSpeed(motorLow);
    			WallFollowingLab.rightMotor.setSpeed(50);
    		    WallFollowingLab.leftMotor.forward();
    		    WallFollowingLab.rightMotor.forward();	
    		}
    }
    //case 3: the robot is too far from the wall
    else if (difference > 0 ){
    		WallFollowingLab.leftMotor.setSpeed(motorLow);
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
	    WallFollowingLab.leftMotor.forward();
	    WallFollowingLab.rightMotor.forward();
    }   
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
