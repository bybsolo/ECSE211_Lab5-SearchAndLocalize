package ca.mcgill.ecse211.lab5;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
/**
 * This class is used for localizing the robot using only ultrasonic sensor
 * @author team12
 */

public class UltrasonicLocalizer {
	private static final double D = 45; //D value for wall detection
	private static final int DETECT_SPEED = Lab5.DETECT_SPEED;
	private static final double WHEEL_RAD = Lab5.WHEEL_RAD; 
	private static final double TRACK = Lab5.TRACK;  
	
	/**
	 * This is the falling edge method used when the robots starts facing away from the wall (distance larger than D) 
	 * It will first turn clockwise to detect the back wall (angle: alpha)
	 * then counter-clockwise to detect the left wall (angle: beta)
	 * @param usDistance the sensor data sampler
	 * @param usData the sensor data buffer
	 * @param odometer the odometer used to determine the robots orientation
	 * @param leftMotor the left motor of the robot
	 * @param rightMotor the right motor of the robot
	 */
	static void fallingEdge(SampleProvider usDistance, float[] usData, Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		int distance;
		usDistance.fetchSample(usData, 0);
		distance = (int) (usData[0] * 100.0); 
		
		//check is it is indeed facing away from the wall
		if (distance > D) {
		    double alpha =0; //the angle when the back wall is detected
		    double beta =0; //the angle when the left wall is detected
		    
		    //turn clockwise to find the back wall and the corresponding angle alpha
	        //reset the motor
		  	for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		  	      motor.stop();
			      motor.setAcceleration(3000);
	  		    }
	  		// Sleep for 1 second
	  	    try {
		  	      Thread.sleep(1000);
		    } catch (InterruptedException e) {      // There is nothing to be done here
		  	}    
	  	    leftMotor.setSpeed(DETECT_SPEED); 
	  	    rightMotor.setSpeed(DETECT_SPEED);
	  	    
	  	    //detect alpha
	  	    boolean back_detected = false;
	  	    while(back_detected == false) {
			    leftMotor.forward();
			    rightMotor.backward();
	  	    	
	  	    	usDistance.fetchSample(usData, 0);
	  		    distance = (int) (usData[0] * 100.0); 		    
	  		    if(distance <= D) {
	  		    	alpha = odometer.getXYT()[2];
	  		    	back_detected = true;
	  		    }
	  	    }	  
	  	    
	  	    //turn the robot counter-clockwise out of the alpha detection zone
	  	    leftMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 45), true);
	  	    rightMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 45), false);
	  	    
	  	    //turn counter-clockwise to find the left wall and the corresponding angle beta
	  	    //reset the motor
		  	for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		  	      motor.stop();
			      motor.setAcceleration(3000);
	  		    }
	  		// Sleep for 1 second
	  	    try {
		  	      Thread.sleep(1000);
		    } catch (InterruptedException e) {     
		  	}  
	  	    leftMotor.setSpeed(DETECT_SPEED); 
	  	    rightMotor.setSpeed(DETECT_SPEED);
	  	    
	  	    //detect beta
	  	    boolean left_detected = false;
	  	    while(left_detected == false) {
			    leftMotor.backward();
			    rightMotor.forward();
			    
	  	    	usDistance.fetchSample(usData, 0);
	  		    distance = (int) (usData[0] * 100.0); 		    
	  		    if(distance <= D) {
	  		    	beta = odometer.getXYT()[2];
	  		    	left_detected = true;
	  		    }
	  		 
	  	    }	  
	  	    
	  	    //move the sensor away from the beta detection zone
	  	    leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 45), true);
	  	    rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 45), false);
	  	    
	  	    //reset the motor
		  	for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		  	      motor.stop();
			      motor.setAcceleration(3000);
	  		    }
	  		// Sleep for 1 second
	  	    try {
		  	      Thread.sleep(1000);
		    } catch (InterruptedException e) {      
		  	}	  	    
	  	    leftMotor.setSpeed(DETECT_SPEED); 
	  	    rightMotor.setSpeed(DETECT_SPEED);
	  	    
	  	    //calculate the change in angle and then turn to the adjusted orientation
	  	    //delta is the angle of the real 0 axis when we use initial orientation as 0 axis 
	  	    double delta = (alpha+beta)/2 -219; 
	  	    if (delta<0) delta = 360+delta;
	  	    //turn to the real zero axis
	  	    leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, (delta-odometer.getXYT()[2])), true);
		    rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, (delta-odometer.getXYT()[2])), false);	    
		    
		    //correct the odometer orientation to zero
		    odometer.setTheta(0);
		}
		//invoke rising edge if it is facing the wall; use rising edge instead
		else {
			risingEdge(usDistance, usData, odometer, leftMotor, rightMotor);
		}

	}
	

	/**
	 * This is the rising edge method used when the robots starts facing the wall (distance smaller than D); 
	 * It will first turn clockwise to detect the left wall (angle: beta)
	 * then counter-clockwise to detect the back wall (angle: alpha)
	 * @param usDistance the sensor data sampler
	 * @param usData the sensor data buffer
	 * @param odometer the odometer used to determine the robots orientation
	 * @param leftMotor the left motor of the robot
	 * @param rightMotor the right motor of the robot
	 */	
	static void risingEdge(SampleProvider usDistance, float[] usData, Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
	    int distance;    
	    usDistance.fetchSample(usData, 0);
		distance = (int) (usData[0] * 100.0); 	
		
		//check if indeed is facing the wall, only then use rising edge
        if(distance < D) {
    	    double alpha =0; //the angle when the back wall is detected
    	    double beta =0; //the angle when the left wall is detected
    	    
    	    //turn clockwise to find the left wall and the corresponding angle beta
            //reset the motor
    	  	for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
    	  	      motor.stop();
    		      motor.setAcceleration(3000);
      		    }
      		// Sleep for 1 second
      	    try {
    	  	      Thread.sleep(1000);
    	    } catch (InterruptedException e) {      // There is nothing to be done here
    	  	}
    	    leftMotor.setSpeed(DETECT_SPEED); 
      	    rightMotor.setSpeed(DETECT_SPEED);
      	    
      	    //detect beta
      	    boolean left_detected = false;
      	    while(left_detected == false) {
    		    leftMotor.forward();
    		    rightMotor.backward();
      	    	
      	    	usDistance.fetchSample(usData, 0);
      		    distance = (int) (usData[0] * 100.0); 		    
      		    if(distance >= D) {
      		    	System.out.println("detect 1st rising");
      		    	beta = odometer.getXYT()[2];
      		    	left_detected = true;
      		    }
      	    }	  

      	    
      	    //turn the robot counter-clockwise out of the beta detection zone
      	    leftMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 45), true);
      	    rightMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 45), false);
      	    
      	    //turn counter-clockwise to find the back wall and the corresponding angle alpha
      	    //reset the motor
    	  	for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
    	  	      motor.stop();
    		      motor.setAcceleration(3000);
      		    }
      		// Sleep for 1 second
      	    try {
    	  	      Thread.sleep(1000);
    	    } catch (InterruptedException e) {      
    	  	}	    
      	    leftMotor.setSpeed(DETECT_SPEED); 
      	    rightMotor.setSpeed(DETECT_SPEED);
    	    
      	    //detect alpha
      	    boolean back_detected = false;
      	    while(back_detected == false) {
    		    leftMotor.backward();
    		    rightMotor.forward();
    		    
      	    	usDistance.fetchSample(usData, 0);
      		    distance = (int) (usData[0] * 100.0); 		    
      		    if(distance >= D) {
      		    	System.out.println("detect 2nd rising");
      		    	alpha = odometer.getXYT()[2];
      		    	back_detected = true;
      		    }
      		 
      	    }	  
      	    
      	    //turn the robot counter-clockwise out of the beta detection zone
      	    leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 45), true);
      	    rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 45), false);
      	    
      	    //reset the motor
    	  	for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
    	  	      motor.stop();
    		      motor.setAcceleration(3000);
      		    }
      		// Sleep for 1 second
      	    try {
    	  	      Thread.sleep(1000);
    	    } catch (InterruptedException e) {      
    	  	}	    	    
      	    leftMotor.setSpeed(DETECT_SPEED); 
      	    rightMotor.setSpeed(DETECT_SPEED);
      	    
      	    //calculate the change in angle and then turn to the adjusted orientation
      	    //delta is the angle of the real 0 axis in the system where the original heading was the zero axis.
      	    double delta = (alpha+beta)/2 -41;
      	    if(delta <0) delta = 360+delta;
      	    //turn to the real 0 axis
      	    leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, (delta-odometer.getXYT()[2])), true);
    	    rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, (delta-odometer.getXYT()[2])), false);
    	    
    	    //correct the odometer orientation to zero
    	    odometer.setTheta(0);

        }
        //if the robot is actually facing away from the wall rather than facing the wall, it will correct itself and use fallingEdge instead
		else {
			fallingEdge(usDistance, usData, odometer, leftMotor, rightMotor);
		}

	}
	
	
}
