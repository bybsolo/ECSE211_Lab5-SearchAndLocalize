package ca.mcgill.ecse211.lab5;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

/**
 * This class contains methods for traversing within the search area and searching for the rings 
 * @author Team12
 *
 */
public class Traverse {
	/**
	 * This class is used for the search routine within the search area
	 * The robot starts at the lower left corner 
	 * the robot will travel along the perimeter of the search area 
	 * while looking for the ring using the ultrasonic sensor on it's right side 
	 * once it detects a ring it will turn, approach and then identify the color using the light sensor
	 * after finding the desired ring, it will travel to the upper right corner
	 */
	private static final int DETECT_SPEED = Lab5.DETECT_SPEED;
	private static final int ROTATE_SPEED = Lab5.ROTATE_SPEED;
	private static final int MOVE_SPEED = Lab5.MOVE_SPEED;
	private static final double WHEEL_RAD = Lab5.WHEEL_RAD;
	private static final double TRACK = Lab5.TRACK;
	private static final double TILE_SIZE = Lab5.TILE_SIZE;
	private static final int DETECT_DISTANCE = Lab5.DETECT_DISTANCE;
	private static int ringCount = 0;
	
	private static SampleProvider usDistanceR = Lab5.usDistanceR;
	private static final float[] usDataR = Lab5.usDataR;
	private static SampleProvider usDistance = Lab5.usDistance;
	private static final float[] usData = Lab5.usData;
	private static final EV3LargeRegulatedMotor leftMotor = Lab5.leftMotor;
	private static final EV3LargeRegulatedMotor rightMotor = Lab5.rightMotor;
	
	private static boolean foundTargetRing;//whether the target ring is found 
	
	public static void search(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		//the search routine ends with the robot returning back on the perimeter of the search area
		
		//orient to the real 0 axis at the lower left corner 
		//reset the motor
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(2000);
		}
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, (0 - odometer.getXYT()[2])), true);
		rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, (0 - odometer.getXYT()[2])), false);
		
		//start traveling along the perimeter of the search area clockwise while detecting the objects	
		foundTargetRing = false;
		double[][] corners = {{Lab5.LLx, Lab5.URy},{Lab5.URx, Lab5.URy},{Lab5.URx, Lab5.LLy},{Lab5.LLx, Lab5.LLy}}; //these are the four corners
		int i = 0;
		//as long as the ring is not found, continue to the next corner and keep searching along the way
		while(foundTargetRing ==false && i<4) {
			detectTill(corners[i][0], corners[i][1], odometer);
			i++;
		}	
		
		// if the robot is on the upper or right side of the search area
		//////////CHANGE THE BOOLEAN CONDITIONS////////////
		/////////////////////////////////////////////////////
		if (odometer.getXYT()[2] == 90 || odometer.getXYT()[2] == 180) {
			Navigation.travelTo(Lab5.URx, Lab5.URy, odometer, leftMotor, rightMotor);
		}
		// if the robot is on the left side of the search area
		if (odometer.getXYT()[2] == 0) {
			Navigation.travelTo(Lab5.URx, Lab5.LLy, odometer, leftMotor, rightMotor);
			Navigation.travelTo(Lab5.URx, Lab5.URy, odometer, leftMotor, rightMotor);
		}
		// if the robot is on the lower side of the search area
		if (odometer.getXYT()[2] == 270) {
			Navigation.travelTo(Lab5.LLx, Lab5.URy, odometer, leftMotor, rightMotor);
			Navigation.travelTo(Lab5.URx, Lab5.URy, odometer, leftMotor, rightMotor);
		}
	} 
	
	public static void detectTill(double x, double y, Odometer odometer) {
		System.out.println("start detectTill");
		double currentX = odometer.getXYT()[0]; //get the current x position in cm
		double currentY = odometer.getXYT()[1]; //get the current y position in cm
		double currentT = odometer.getXYT()[2]; //get the current direction in degrees
		
		//calculate the moving distance and turning angle
		double x1 = x*TILE_SIZE; //way point x coordinate in cm
		double y1 = y*TILE_SIZE; //way point y coordinate in cm
//		double dDistance = Math.sqrt(Math.pow((x1 - currentX), 2) + Math.pow((y1 - currentY), 2));
		double dAngle = Navigation.getDAngle(x1, y1, currentX, currentY); //get the angle to turn
		Navigation.turnTo(dAngle, currentT, leftMotor, rightMotor); //turn the robot to the direction of the new way point
		System.out.println("turn robot to direction of waypoint");
		//reset the motor

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.stop();
		      motor.setAcceleration(3000);
		    }
		try {
	      Thread.sleep(1000);
	    } catch (InterruptedException e) {}
	    
	    //move the robot to the way point
	    leftMotor.setSpeed(ROTATE_SPEED);
	    rightMotor.setSpeed(ROTATE_SPEED);
	    System.out.println("set speed to " + ROTATE_SPEED);
	    
//	    leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, dDistance), true);  
//	    rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, dDistance), true);	  
//	    System.out.println("rotate wheels to go distance " + dDistance);
	    boolean foundRing = false;
	    while(foundRing == false) {
	    	double dDistance = Math.sqrt(Math.pow((x1 - currentX), 2) + Math.pow((y1 - currentY), 2));
	    	leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, dDistance), true);  
		    rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, dDistance), true);	  
		    //System.out.println("rotate wheels to go distance " + dDistance);
			usDistanceR.fetchSample(usDataR, 0);
			int distance = (int)(usDataR[0]*100.0);
			System.out.println(distance);
			if(distance < DETECT_DISTANCE) {
				ringCount++;
			}
			else {
				ringCount = 0;
			}
			//turn and approach if detected
			if(ringCount > 5) {
				System.out.println("..........................detect Ring");
				Sound.beep();
				Sound.beep();
				foundRing = true;
				detect(x, y, odometer);
			}
		}
	}
	
	public static void detect(double x, double y, Odometer odometer) {
		//rotate back a bit, since there is a offset between the right side ultrasonic sensor and wheel sensor
		// reset the motor
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {}
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, Lab5.OFF_SET_R), true);
		rightMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, Lab5.OFF_SET_R), false);
		
		//turns 90degrees
		// reset the motor
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 90), true);
		rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 90), false);
		
		//record the odometer x y values at this found so we can return to it later 
		double xRecord = odometer.getXYT()[0];
		double yRecord = odometer.getXYT()[1];
		
		//and slowly moves forward
		// reset the motor
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {}
		leftMotor.setSpeed(DETECT_SPEED); //change this sped if its too slow
		rightMotor.setSpeed(DETECT_SPEED);
		boolean detected = false;
		while(detected == false) {
			leftMotor.forward();
			rightMotor.forward();
			usDistance.fetchSample(usData, 0);
			int dis = (int)(usData[0] * 100.0);
			if(dis<Lab5.RING_BAND) {
				int color = Color.color();
				detected = true;
				if (color == Lab5.TR) {
					System.out.println("color matched");
					Sound.beep();
					Sound.beep();
					foundTargetRing = true; //we have found the ring! this boolean will now terminated the biggest while loop in search()
				} else Sound.beep();
			}
		}
		
		//back off to the perimeter (where the robot was before turning entering the search field)
		// reset the motor
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		double disReturn = Math.sqrt(Math.pow((odometer.getXYT()[0]- xRecord), 2)+Math.pow((odometer.getXYT()[1]- yRecord), 2)); //this is how much it needs
		leftMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, disReturn), true);
		rightMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, disReturn), false);
		
		//call detectTill again, so it can keep detecting more rings (recursive)
		if (foundTargetRing == false) detectTill(x, y, odometer);
	}
}
