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
	
	private static boolean foundTargetRing;//return true when the target ring is found 
	
	/**
	 * this method is used for searching the search area for the rings,
	 * it is called in the main method, and will search for rings (including the target ring) while traversing around the 
	 * search area perimeter
	 * @param leftMotor the left motor of the robot
	 * @param rightMotor the right motor
	 * @param odometer the odometer used bu the robot
	 */
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
		
		//update thr odometer values based on the starting corner
		if(Lab5.SC == 0 || Lab5.SC == 1) odometer.setXYT(Lab5.LLx*TILE_SIZE, Lab5.LLy*TILE_SIZE, 0);
		if(Lab5.SC == 2 || Lab5.SC == 3) odometer.setXYT(Lab5.LLx*TILE_SIZE, Lab5.LLy*TILE_SIZE, 0);
		//orient to the 0 axis
		leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, (0 - odometer.getXYT()[2])), true);
		rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, (0 - odometer.getXYT()[2])), false);
		
		//start traveling along the perimeter of the search area clockwise while detecting the objects	
		foundTargetRing = false;
		double[][] corners = {{Lab5.LLx, Lab5.URy},{Lab5.URx, Lab5.URy},{Lab5.URx, Lab5.LLy},{Lab5.LLx, Lab5.LLy}}; //these are the four corners
		int i = 0;
		//as long as the ring is not found, continue to the next corner and keep searching along the way
		while(foundTargetRing ==false && i<4) {
			detectTill(corners[i][0], corners[i][1], odometer);
			System.out.println("reached corner");
			i++;
		}	
		
		//if the ring is found in the end/ the end of the perimeter (the starting point) is reached, travel to the upper right corner
		// if the robot is on the upper or right side of the search area
		double currentT = odometer.getXYT()[2];
		if ((currentT >= 80 && currentT <= 100) || (currentT >= 170 && currentT <= 190)) {
			Navigation.travelTo(Lab5.URx, Lab5.URy, odometer, leftMotor, rightMotor);
		}
		// if the robot is on the left side of the search area
		if ((currentT >= 0 && currentT <= 10) || (currentT >= 350 && currentT <= 360)) {
			Navigation.travelTo(Lab5.LLx, Lab5.URy, odometer, leftMotor, rightMotor);
			Navigation.travelTo(Lab5.URx, Lab5.URy, odometer, leftMotor, rightMotor);
		}
		// if the robot is on the lower side of the search area
		if ((currentT >= 260) && currentT <= 280) {
			Navigation.travelTo(Lab5.URx, Lab5.LLy, odometer, leftMotor, rightMotor);
			Navigation.travelTo(Lab5.URx, Lab5.URy, odometer, leftMotor, rightMotor);
		}
	} 
	
	/**
	 * 
	 * @param x the x coordinates (in unit tile-length) of the corner we are traveling to 
	 * @param y the y coordinate  (in unit tile-length) of the corner we are traveling to 
	 * @param odometer the odometer used
	 */
	public static void detectTill(double x, double y, Odometer odometer) {
		System.out.println("start detectTill going to "+x+", "+y);
		double currentX = odometer.getXYT()[0]; //get the current x position in cm
		double currentY = odometer.getXYT()[1]; //get the current y position in cm
		double currentT = odometer.getXYT()[2]; //get the current direction in degrees
		
		//calculate the moving distance and turning angle
		double x1 = x*TILE_SIZE; //way point x coordinate in cm
		double y1 = y*TILE_SIZE; //way point y coordinate in cm
		double dAngle = Navigation.getDAngle(x1, y1, currentX, currentY); //get the angle to turn
		Navigation.turnTo(dAngle, currentT, leftMotor, rightMotor); //turn the robot to the direction of the new way point
		System.out.println("turn robot to direction of waypoint");
		//reset the motor
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.setAcceleration(3000);
		    }
		try {
	      Thread.sleep(1000);
	    } catch (InterruptedException e) {}
	    
	    //move the robot to the way point
	    leftMotor.setSpeed(ROTATE_SPEED);
	    rightMotor.setSpeed(ROTATE_SPEED);
	    System.out.println("set speed to " + ROTATE_SPEED);
	  
	    boolean foundRing = false;
	    while(foundRing == false) {
	    	currentX = odometer.getXYT()[0];
	    	currentY = odometer.getXYT()[1];
	    	double dDistance = Math.sqrt(Math.pow((x1 - currentX), 2) + Math.pow((y1 - currentY), 2));
	    	if (dDistance <= 2) {
	    		break;
	    		}
	    	leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, dDistance), true);  
		    rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, dDistance), true);	  
			usDistanceR.fetchSample(usDataR, 0);
			int distance = (int)(usDataR[0]*100.0);
			System.out.println(distance);
			double xPos = odometer.getXYT()[0];
			double yPos = odometer.getXYT()[1];
			if( (Math.abs(xPos - x ) <= 0.2) && (Math.abs(yPos - y) <= 1 )
				
			||( (Math.abs(yPos - y) <= 0.2) && (Math.abs(xPos - x) <= 1 ) ) 
			) {
				System.out.println("skip reading");
				Sound.beep();
;				continue;
				
			}
			
			if(distance < DETECT_DISTANCE) {
				ringCount++;
			}
			else {
				ringCount = 0;
			}
			//turn and approach if detected
			if(ringCount >= 10) {
				Sound.beep();
				Sound.beep();
				foundRing = true;
				double correction = 0;
				currentY= odometer.getXYT()[1];
				currentT = odometer.getXYT()[2];
				System.out.println("current Y is " + odometer.getXYT()[1]);
				System.out.println("current X is " + odometer.getXYT()[0]);
				System.out.println("angle is " + odometer.getXYT()[2]);
				//going up
				if ((currentT >= 0 && currentT <= 10) ||(currentT >= 350 && currentT <= 360) ) {
					correction = TILE_SIZE - (currentY % TILE_SIZE);						
				}
				//going right
				else if ((currentT >= 80 && currentT <= 110)  ) {
					correction = TILE_SIZE - (currentX % TILE_SIZE);	
				}
				//going down
				else if ((currentT >= 170 && currentT <= 190)  ) {
					correction = currentY % TILE_SIZE;
				}
				//going left
				else  {
					correction = currentX % TILE_SIZE;
				} 
				System.out.println("corrention is "+ correction);
				leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, correction), true);  
			    rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, correction), false);
			    try {
					Thread.sleep(500);
				} catch (InterruptedException e) {}
				detect(x, y, odometer);
				
			}
		}
	}
	
	/**
	 * this method is used when the right ride sensor detects the ring and it will enter the searching area and search/idnetify the ring
	 * @param x the x coordinate (in unit tile-length) of the corner we are traveling to 
	 * @param y the y coordinate (in unit tile-length) of the corner we are traveling to 
	 * @param odometer the odometer object used
	 */
	public static void detect(double x, double y, Odometer odometer) {
		int detectCount = 0;
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {}
		
		//record the odometer x y values at this found so we can return to it later 

		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 90), true);
		rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 90), false);
		//record the position for returning distance calculation 
		double xRecord = odometer.getXYT()[0];
		double yRecord = odometer.getXYT()[1];
		double disReturn;
		
		//and slowly moves forward
		// reset the motor
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {}
		leftMotor.setSpeed(ROTATE_SPEED); //change this sped if its too slow
		rightMotor.setSpeed(ROTATE_SPEED);
		boolean detected = false;
		while(detected == false) {
			disReturn = Math.sqrt(Math.pow((odometer.getXYT()[0]- xRecord), 2)+Math.pow((odometer.getXYT()[1]- yRecord), 2));
			if(disReturn > TILE_SIZE *2) {
				System.out.println("false detection go back");
				detected = true;
				break;
			}
			//fetch ultrasonic front readings.
			usDistance.fetchSample(usData, 0);
			int dis = (int)(usData[0] * 100.0);
			if(dis < Lab5.RING_BAND) {
				detectCount++;
			}
			else {
				detectCount = 0;
			}
			if(detectCount >= 7) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
					motor.setAcceleration(3000);
				}
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {}
				leftMotor.setSpeed(DETECT_SPEED);
				rightMotor.setSpeed(DETECT_SPEED);
				leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, Lab5.RING_BAND - 6), true);
				rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, Lab5.RING_BAND - 6), false);
				int color = Color.color();
				detected = true;
				if (color == Lab5.TR) {
					System.out.println("color matched");
					Sound.beep();
					Sound.beep();
					foundTargetRing = true; //we have found the ring! this boolean will now terminated the biggest while loop in search()
				} 
				else if(color == 0) {
					System.out.println("failed to identify");
					detected = false;
				}
				else Sound.beep();
			}
			leftMotor.forward();
			rightMotor.forward();
		}
		
		//back off to the perimeter (where the robot was before turning entering the search field)
		// reset the motor
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		disReturn = Math.sqrt(Math.pow((odometer.getXYT()[0]- xRecord), 2)+Math.pow((odometer.getXYT()[1]- yRecord), 2)); //this is how much it needs
		leftMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, disReturn ), true);
		rightMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, disReturn ), false);
		
		//call detectTill again, so it can keep detecting more rings (recursive)
		if (foundTargetRing == false) {
			// reset the motor
			for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
				motor.stop();
				motor.setAcceleration(3000);
			}
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {}
			leftMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 91), true);
			rightMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 91), false);
			// reset the motor
			for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
				motor.stop();
				motor.setAcceleration(3000);
			}
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {}
			leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
			rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), false);
			detectTill(x, y, odometer);
		}		
		else {
			leftMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 90), true);
			rightMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 90), false);
		}
	}
}
