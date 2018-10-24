package ca.mcgill.ecse211.lab5;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class is used for localizing the robot using the light sensor
 * The robot will first move closer to (0,0) but still well within the 3rd quadrant (details see the method comment)
 * During the sampling, line detection at 1 and 3 are intersections with x axis, the odometer orientation
 * readings are used for determining the y position at lines 2 and 4 the sensor
 * is intersecting the y axis, the odometer orientation readings are used for
 * determining the x position The orientation when crossing line 4 is used for
 * another calibration of the orientation
 * 
 * @author Team12
 */

public class LightLocalizer {
	private static final int THRESHOLD = 300;
	private static final double SAFE_DISTANCE = 5 ;
	
	private static final double OFF_SET = Lab5.OFF_SET;
	private static final int ROTATE_SPEED = Lab5.ROTATE_SPEED;
	private static final int MOVE_SPEED = Lab5.MOVE_SPEED;
	private static final double WHEEL_RAD = Lab5.WHEEL_RAD;
	private static final double TRACK = Lab5.TRACK;
	private static final double TILE_SIZE = Lab5.TILE_SIZE;
	
	private static final Port portLine = Lab5.portLine;
	private static final SensorModes myLine = Lab5.myLine;
	private static final SampleProvider myLineSample = Lab5.myLineSample;
	private static final float[] sampleLine = Lab5.sampleLine;


	/**
	 * The method for localizing the robot using the light sensor
	 * Before starting the light localization it will move the robot closer to (0,0) but still within the 3rd quadrant 
	 * by letting it move closer and detect the X and Y axis then back off a bit to a safe distance to ensure the light sensor will later
	 * detect the lines in the correct order when sampling
	 * @param myColorSample the light sensor data sampler
	 * @param sampleColor   the light sensor data buffer
	 * @param odometer      the odometer used by the robot
	 * @param leftMotor     the left motor of the robot
	 * @param rightMotor    the right motor of the robot
	 * @throws OdometerExceptions
	 */
	static void lightLocalize(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor)
			throws OdometerExceptions {
		
		// move the robot closer to the (0,0) to make sure the light sensor will be able
		// to detect the lines in the correct order in the sampling process
		// this is achieved by letting the robot approach each axis then back off a bit
		//in the end guarantee the robot is within the 3rd quadrant and close enough to (0,0) before light localization starts.
		
		//turn the robot by 90 degrees clockwise and move forward, after detecting the Y axis it will back off a bit to a safe distance
		//to ensure the light sensor will detect the lines in the right order in the sampling process
		
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
		//move the robot 
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 90),true);
		rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 90),false);
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
		leftMotor.setSpeed(MOVE_SPEED);
		rightMotor.setSpeed(MOVE_SPEED);
		boolean yAxis = false;
		while (yAxis == false) {
			myLineSample.fetchSample(LightLocalizer.sampleLine, 0); //get the reading from the sensor
		    float read = LightLocalizer.sampleLine[0]*1000;  //multiply the read by 1000 as suggested in the class slides
			leftMotor.forward();
			rightMotor.forward();
		    if (read<THRESHOLD) yAxis = true;
		}
		leftMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, SAFE_DISTANCE+OFF_SET),true);
		rightMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, SAFE_DISTANCE+OFF_SET),false);
		
		//turn the robot counter-clockwise by 90 degrees, move forward to detect the X axis 
		//and then move back a little to a safe distance to ensure the light sensor will detect the lines in the correct order
		//in the sampling process
		
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
		//turn the robot by 90 degrees counter-clockwise
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 90),true);
		rightMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 90),false);
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
		//move the robot forward until the x axis is detected
		leftMotor.setSpeed(MOVE_SPEED);
		rightMotor.setSpeed(MOVE_SPEED);
		boolean xAxis = false;
		while (xAxis == false) {
			myLineSample.fetchSample(LightLocalizer.sampleLine, 0); //get the reading from the sensor
		    float read = LightLocalizer.sampleLine[0]*1000;  //multiply the read by 1000 as suggested in the class slides
		    leftMotor.forward();
			rightMotor.forward();		
			if (read<THRESHOLD) xAxis = true;
		}
		leftMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, SAFE_DISTANCE+OFF_SET),true);
		rightMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, SAFE_DISTANCE+OFF_SET),false);
		
		//start the sampling process
		Sampling sampling = new Sampling(); // start the light sensor localization sampling process
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

		// rotate the robot clockwise to allow the light sensor to scan across the lines
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		// start sampling, as sampling starts, turn the robot by 360
		Thread samplingThread = new Thread(sampling);
		samplingThread.start();
		leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 360), true);
		rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 360), false);

		// label the samples
		double xN = sampling.odoReading[0];
		double yP = sampling.odoReading[1];
		double xP = sampling.odoReading[2];
		double yN = sampling.odoReading[3]; // this is the 4th crossing, will be used for angle correction
		double xAngle = xP - xN;
		double yAngle = yN - yP;

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

		// the calculation for x, y
		double x = -OFF_SET * Math.cos(Math.toRadians(yAngle / 2));
		double y = -OFF_SET * Math.cos(Math.toRadians(xAngle / 2));
		// calculate the angle; we do it twice using 2 angles and take the average
		// delta is the angle of the real 0 axis in a system where the original heading
		// is the 0 axis
		double delta1 = yN - yAngle / 2 - 270;
		if (delta1 < 0)
			delta1 = delta1 + 360;
		double delta2 = xP - xAngle / 2 - 180;
		if (delta2 < 0)
			delta2 = delta2 + 360;
		double delta = (delta1 + delta2) / 2;

		// turn the robot to the real 0 axis and reset the x, y, theta value;
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, delta), true);
		rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, delta), false);
		odometer.setXYT(x, y, 0);

		// move the robot to the origin and orient to the real 0 axis
		Navigation.travelTo(0, 0, odometer, leftMotor, rightMotor);
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, (0 - odometer.getXYT()[2])), true);
		rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, (0 - odometer.getXYT()[2])), false);
		
		if(Lab5.SC == 0) odometer.setXYT(TILE_SIZE, TILE_SIZE, 0);
		if(Lab5.SC == 1) odometer.setXYT(7*TILE_SIZE, TILE_SIZE, 270);
		if(Lab5.SC == 2) odometer.setXYT(7*TILE_SIZE, 7*TILE_SIZE, 180);
		if(Lab5.SC == 3) odometer.setXYT(TILE_SIZE, 7*TILE_SIZE, 90);
	}

}
