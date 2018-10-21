package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This method is used for sampling in line detection involved in light localization (using the light sensor)
 * it will run on its own thread and have a sample period of 10
 * @author Team12
 *
 */
public class Sampling implements Runnable{
	private static final long SAMPLE_PERIOD =12;
	private static final int THRESHOLD = 400;
	private Odometer odometer;
	public double[] odoReading = new double[4];
	
	/**
	 * constructor for the Sampling
	 * @throws OdometerExceptions
	 */
	public Sampling() throws OdometerExceptions{
		this.odometer = Odometer.getOdometer();
		this.odoReading = new double[4];
	}
	
	/**
	 * this method overrides the run() in runnable to create a sampler than has a sampling period
	 * it will run on its own thread
	 */
	@Override
	public void run() {
		long sampleStart, sampleEnd;
		int sampleCount = 0;
		while(sampleCount<4) {
			sampleStart = System.currentTimeMillis();
		   
		    Lab5.myLineSample.fetchSample(Lab5.sampleLine, 0); //get the reading from the sensor
		    float read = Lab5.sampleLine[0]*1000;  //multiply the read by 1000 as suggested in the class slides
		    //the threshold for a successful line detection is 400 		    
		    if(read<THRESHOLD) {
		    	odoReading[sampleCount] = odometer.getXYT()[2];
		    	sampleCount ++;
		    	Sound.beep();
		    }
		    sampleEnd  = System.currentTimeMillis();
		    if (sampleEnd - sampleStart < SAMPLE_PERIOD) {
		      try {
		        Thread.sleep(SAMPLE_PERIOD - (sampleEnd - sampleStart));
		      } catch (InterruptedException e) {
		        // there is nothing to be done here
		      }
		    }
		    

		}
		
	}
	

}