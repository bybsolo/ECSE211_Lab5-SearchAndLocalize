package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.lcd.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class contains methods needed to perform all necessary tests and calibrations 
 * including color calibration and the driver
 * @author Team12
 *
 */
public class Tester {
	public static final TextLCD lcd = Lab5.lcd;
	public static final Port portColor = Lab5.portColor; // get the port for the light (color sensor)
	public static final SensorModes myColor = Lab5.myColor; // create the color sensor object;
	public static final SampleProvider myColorSample = Lab5.myColorSample;
	public static final float[] sampleColor = Lab5.sampleColor; // create an array for the sensor reading
	public static final EV3LargeRegulatedMotor leftMotor = Lab5.leftMotor;
	public static final EV3LargeRegulatedMotor rightMotor = Lab5.rightMotor;
	
	public static void sample() {
		lcd.clear();
		int counter =0;
		while(counter<300) {
			myColorSample.fetchSample(sampleColor, 0); 
			float r = sampleColor[0]*1000; 
			float g = sampleColor[1]*1000; 
			float b = sampleColor[2]*1000; 
			System.out.print(r +",");
			System.out.print(g+",");
			System.out.println(b);
			counter ++;
		}		
	}
	
	/**
	 * This method calibrates the wheel radius value of the robot
	 * since the method convertDistance only uses wheel radius
	 * @param leftMotor left motor of the robot
	 * @param rightMotor right motor of the robot 
	 */
	public static void wheelRadCheck() {
		// reset the motor
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(2000);
		}
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}
		//move the robot forward until the Y asis is detected
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		leftMotor.rotate(Navigation.convertDistance(Lab5.WHEEL_RAD, 2*Lab5.TILE_SIZE), true);
		rightMotor.rotate(Navigation.convertDistance(Lab5.WHEEL_RAD, 2*Lab5.TILE_SIZE), false);
	}
	
	/**
	 * This method calibrates the wheelbase value of the robot once the 
	 * wheel radius is calibrated
	 * @param leftMotor left motor of the robot
	 * @param rightMotor right motor of the robot 
	 */
	public static void trackCheck() {
		// reset the motor
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(2000);
		}
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}
		//move the robot forward until the Y asis is detected
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.rotate(Navigation.convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 360), true);
		rightMotor.rotate(-Navigation.convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 360), false);
	}
	
	public static void usSample() {
		System.out.println("start US sampling");
		int ringCount = 0;
		SampleProvider usDistanceR = Lab5.usDistanceR;
		float[] usDataR = Lab5.usDataR;
		// reset the motor
//				for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
//					motor.stop();
//					motor.setAcceleration(2000);
//				}
//				try {
//					Thread.sleep(1000);
//				} catch (InterruptedException e) {
//					// There is nothing to be done here
//				}
				//move the robot forward until the Y asis is detected
//				leftMotor.setSpeed(100);
//				rightMotor.setSpeed(100);
				boolean foundRing = false;
				leftMotor.setSpeed(100);
		    	rightMotor.setSpeed(100);
			    while(foundRing == false) {
//			    	double dDistance = Math.sqrt(Math.pow((x1 - currentX), 2) + Math.pow((y1 - currentY), 2));
//			    	leftMotor.rotate(Navigation.convertDistance(Lab5.WHEEL_RAD, dDistance), true);  
//				    rightMotor.rotate(Navigation.convertDistance(Lab5.WHEEL_RAD, dDistance), true);	  
//				    System.out.println("rotate wheels to go distance " + dDistance);
			    	leftMotor.forward();
			    	rightMotor.forward();
					usDistanceR.fetchSample(usDataR, 0);
					int distance = (int)(usDataR[0]*100.0);
					System.out.println(distance);
					if(distance < Lab5.DETECT_DISTANCE) {
						ringCount++;
					}
					else {
						ringCount = 0;
					}
					//turn and approach if detected
					if(ringCount > 10) {
						System.out.println("..........................detect Ring");
						Sound.beep();
						Sound.beep();
						foundRing = true;
						//detect(x, y, odometer);
					}
				};
	}
}
