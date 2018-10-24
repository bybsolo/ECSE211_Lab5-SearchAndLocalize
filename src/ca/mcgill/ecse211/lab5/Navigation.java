package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * this class is used for navigating the robot to a specific point on the grid (coordinates in cm)
 * @author Team12
 *
 */
public class Navigation {
	
	  private static final int FORWARD_SPEED = Lab5.MOVE_SPEED; 
	  private static final int ROTATE_SPEED = Lab5.ROTATE_SPEED;
	  private static final double WHEEL_RAD = Lab5.WHEEL_RAD;
	  private static final double TRACK = Lab5.TRACK;
	  private static final double TILE_SIZE = Lab5.TILE_SIZE;
	
	/**
	 * This method is used to drive the robot to the destination point which is
	 * marked as an absolute coordinate (X, Y) 
	 * The method constantly calls the turnTo method to first adjust to the angle
	 * it needs to turn to before moving
	 * @param x the absolute x-coordinate of the destination, an integer value in double format
	 * @param y the absolute y-coordinate of the destination, an integer value in double format
	 * @param odometer the odometer object created in the main class
	 * @param leftMotor the left motor of the robot 
	 * @param rightmotor the right motor of the robot
	 */
	  	  
	public static void travelTo(double x, double y, Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		
		//get the odometer readings to determine the action
		double currentX = odometer.getXYT()[0]; //get the current x position in cm
		double currentY = odometer.getXYT()[1]; //get the current y position in cm
		double currentT = odometer.getXYT()[2]; //get the current direction in degrees
		
		//calculate the moving distance and turning angle
		double x1 = x*TILE_SIZE; //waypoint x coordinate in cm
		double y1 = y*TILE_SIZE; //waypoint y coordinate in cm
		double dDistance = Math.sqrt(Math.pow((x1 - currentX), 2) + Math.pow((y1 - currentY), 2));
		
		double dAngle = getDAngle(x1, y1, currentX, currentY);
		
		// reset the motor
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
		}
		
		turnTo(dAngle, currentT, leftMotor, rightMotor); //turn the robot to the direction of the new way point
		
		//move the robot towards the new way point
		//reset the motor
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.setAcceleration(3000);
		    }
	    try {
	      Thread.sleep(500);
	    } catch (InterruptedException e) {
	    }

	    leftMotor.setSpeed(FORWARD_SPEED);
	    rightMotor.setSpeed(FORWARD_SPEED);

	    leftMotor.rotate(convertDistance(WHEEL_RAD, dDistance), true);
	    rightMotor.rotate(convertDistance(WHEEL_RAD, dDistance), false);	    
		
	}
	
	/**
	 * 
	 * @param dAngle the angle to turn towards
	 * @param currentT the current direction
	 * @param leftMotor the left motor
	 * @param rightMotor the right motor
	 */
	public static void turnTo (double dAngle, double currentT, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor ) {
		//reset the motor
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.setAcceleration(2000);
		    }
	    try {
	      Thread.sleep(1000);
	    } catch (InterruptedException e) {
	      // There is nothing to be done here
	    }
	    
	    //find the smallest angle to turn to the destination
	    double angle1 = dAngle - currentT;
	    double angle2 = (angle1>=0 ? -(360-(Math.abs(angle1))) : (360-(Math.abs(angle1))));
	    double angle = (Math.abs(angle1) < Math.abs(angle2) ? angle1 : angle2); 
	    
//	    if(angle>=80 && angle<=100) angle = 90;
//	    if(angle>=-10 && angle<=10) angle = 0;
//	    if(angle>=170 && angle<=190) angle = 180;
//	    if(angle>=-100 && angle<=-80) angle = -90;
//	    if(angle>=-190 && angle<=-170) angle = -180;
//	    if(angle>=350 && angle<=361) angle = 0;
	    
	    //start the motors and make the turn
	    leftMotor.setSpeed(ROTATE_SPEED);
	    rightMotor.setSpeed(ROTATE_SPEED);

	    leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, angle), true);
	    rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, angle), false);
	    
	}
	
	/**
	 * 
	 * @param x1 the x of way point
	 * @param y1 the y of way point
	 * @param xc the current x
	 * @param yc the current y
	 * @return the angle to turn in degrees
	 */
	public static double getDAngle(double x1, double y1, double xc, double yc) {
		double xr = x1 - xc;
		double yr = y1 - yc;
		
		//make the angle within 0 to 360
		if(xr == 0 && yr!=0) {
			if(yr>0) return 0; 
			else return 180;
		}
		if(xr != 0 && yr==0) {
			if(xr>0) return 90;
			else return 270;
		}
		if(xr != 0 && yr!=0) {
			if(xr>0 && yr>0) return Math.toDegrees(Math.atan(xr/yr));
			if(xr>0 && yr<0) return Math.toDegrees(Math.atan(Math.abs(yr/xr)))+90;
			if(xr<0 && yr<0) return Math.toDegrees(Math.atan(xr/yr))+180;
			if(xr<0 && yr>0) return Math.toDegrees(Math.atan(Math.abs(yr/xr)))+270;;
		}
		return 0;
	}
	
	/**
	 * converts distance to angle the wheel needs to turn in deg
	 * @param radius of the robot
	 * @param distance of the robot
	 * @return deg to turn
	 */
	public static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	  }
	
	/**
	 * converts angle to the actual angle needs to turn 
	 * @param radius radius of the robot 
	 * @param width radius of the car
	 * @param angle to turn
	 * @return angle that need to turn in deg
	 */
	public static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	    
	    
	  }
}
