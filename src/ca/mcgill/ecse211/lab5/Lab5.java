package ca.mcgill.ecse211.lab5;

import lejos.hardware.sensor.*;
import lejos.hardware.Button;
import ca.mcgill.ecse211.lab5.LightLocalizer;
import ca.mcgill.ecse211.lab5.OdometerExceptions;
import ca.mcgill.ecse211.lab5.Odometer;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

/**
 * this is the main class for Lab 5; 
 * it contains almost all the parameters/sensor initiations 
 * it will interact with the user through the EV3 module's display and buttons
 * to ask the user whether to initiate testing routine or the demo routine.
 * The routines are performed as consecutive methods within threads
 * @author Team12
 *
 */
public class Lab5 {
	//These are the inputs, will be provided before the demo, re-enter them before running
	//0,3,3,7,7 are defaults for report testing
	public static final int SC = 0;
	public static final int TR = 1;
	public static final double LLx = 3;		//left lower x
	public static final double LLy = 3;		//left lower y
	public static final double URx = 7;		//upper right x
	public static final double URy = 7;		//upper right y
	
	// The parameters for driving the robot
	public static final double OFF_SET = 14.65; //this is the offset from the back line-detecting light sensor to the wheelbase
	public static final double OFF_SET_R = 4; //this is the offset from the right side ultrasonic sensor to the wheelbase
	public static final int DETECT_SPEED = 50; //this is the slow speed for precious detection 
	public static final int ROTATE_SPEED = 100;
	public static final int MOVE_SPEED = 200; 
	public static final double WHEEL_RAD = 2.13; 
	public static final double TRACK = 14.5;
	public static final double TILE_SIZE = 30.48;
	public static final int DETECT_DISTANCE = (int)(2.5*TILE_SIZE); //detection bandcenter for the right side ultrasonic sensor /// ahmed: you can modify this 
	public static final int RING_BAND = 20; //detection bandcenter for moving lose up to the ring for color identification 
	
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	
	//create port and object for the light sensor for line detection in the back
	public static final Port portLine = LocalEV3.get().getPort("S4"); // get the port for the light (color sensor)
	public static final SensorModes myLine = new EV3ColorSensor(portLine); // create the color sensor object;
	public static final SampleProvider myLineSample = myLine.getMode("Red");
	public static final float[] sampleLine = new float[myLine.sampleSize()]; // create an array for the sensor
																				// readings	
	// create port and object for the light sensor for color recognition in the front
	public static final Port portColor = LocalEV3.get().getPort("S1"); // get the port for the light (color sensor)
	public static final SensorModes myColor = new EV3ColorSensor(portColor); // create the color sensor object;
	public static final SampleProvider myColorSample = myColor.getMode("RGB");
	public static final float[] sampleColor = new float[3]; // create an array for the sensor
																				// readings
	// create port and object for the ultrasonic sensor in the front (used for ultrasonic localization and when approaching the ring)
	public static final Port usPort = LocalEV3.get().getPort("S3");
	@SuppressWarnings("resource") // Because we don't bother to close this resource
	public static SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
	public static SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
	public static final float[] usData = new float[usDistance.sampleSize()];

	// create port and object for the ultrasonic sensor on the right side of the robot for detecting the ring (denoted by R)
	//when traveling around the search area perimeter
	public static final Port usPortR = LocalEV3.get().getPort("S2");
	@SuppressWarnings("resource") // Because we don't bother to close this resource
	public static SensorModes usSensorR = new EV3UltrasonicSensor(usPortR); // usSensor is the instance
	public static SampleProvider usDistanceR = usSensorR.getMode("Distance"); // usDistance provides samples from
	public static final float[] usDataR = new float[usDistanceR.sampleSize()];
	
	public static void main(String[] args) throws OdometerExceptions{
		final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		int buttonChoice;
	
		do {
			lcd.clear(); //clear the display
			//prompt the user to select whether to run tests (press up), demo color classification (press left),
			//or perform the search and localize procedure
			lcd.drawString("^ tests", 0, 1);
			lcd.drawString("< Color", 0,2); 
		    lcd.drawString("> SandL", 0,3);
		    buttonChoice = Button.waitForAnyPress(); 
		} while (buttonChoice!=Button.ID_UP && buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		
		//initiate the tests if pressed up
		if (buttonChoice ==Button.ID_UP) {
			int testChoice;
			do {
				lcd.clear();
				lcd.drawString("^ Color", 0, 1);
				lcd.drawString("< Radius", 0,2);
			    lcd.drawString("> Track", 0,3);
			    lcd.drawString("US sampling", 0,4);
				testChoice = Button.waitForAnyPress();
			}while(testChoice!=Button.ID_UP && testChoice != Button.ID_LEFT && testChoice != Button.ID_RIGHT && testChoice != Button.ID_DOWN);
			if(testChoice==Button.ID_LEFT) {
				//initiate the wheel radius tuning 
				lcd.clear();
				(new Thread() {
					public void run() {
						Tester.wheelRadCheck();
					}
				}).start();
			}
			if(testChoice==Button.ID_RIGHT) {
				//initiate the wheel base tuning 
				lcd.clear();
				(new Thread() {
					public void run() {
						Tester.trackCheck();
					}
				}).start();

			}
			if(testChoice==Button.ID_UP) {
				//initiate the color data collection
				lcd.clear();
				(new Thread() {
					public void run() {
						Tester.sample();
					}
				}).start();
			}
			if(testChoice==Button.ID_DOWN) {
				System.out.println("press down");
				lcd.clear();
				(new Thread() {
					public void run() {
						Tester.usSample(odometer);
					}
				}).start();
			}
		    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		    System.exit(0);
		}
		
		//initiate demo color calibration is left is pressed
		if(buttonChoice == Button.ID_LEFT) {
			// initiate the color data collection
			lcd.clear();
			(new Thread() {
				public void run() {
					try {
						Color.colorDemo();
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}).start();
		}
		
		//initiate the search and localize if right is pressed
		if(buttonChoice == Button.ID_RIGHT) {
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			
			(new Thread() {
				public void run() {
					//The following are the routine performed for search and localize
					//perform the localization routine
					//UltrasonicLocalizer.risingEdge(usDistance, usData, odometer, leftMotor, rightMotor);
					try {
						System.out.println("....................light localization");
						LightLocalizer.lightLocalize(odometer, leftMotor, rightMotor);
					} catch (OdometerExceptions e) {
						e.printStackTrace();
					}	
					//navigate to the lower left corner of the search area
					odometer.setXYT(TILE_SIZE, TILE_SIZE, 0.0); ///delete this later AHHHHHHHH
					Navigation.travelTo(Lab5.LLx, odometer.getXYT()[1]/TILE_SIZE, odometer, leftMotor, rightMotor); //travel to takes integer coordinates as doubles 
					Navigation.travelTo(Lab5.LLx, Lab5.LLy, odometer, leftMotor, rightMotor);
					Sound.beep();
					System.out.println("......................start search");
					Traverse.search(leftMotor, rightMotor, odometer);
				}
			}).start();
	
		}
		
		//stop the system when the exit button is pressed
	    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
	    System.exit(0);
	}

}
