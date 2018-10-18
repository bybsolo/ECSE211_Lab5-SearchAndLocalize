package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.lcd.*;

public class tester {
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final Port portColor = LocalEV3.get().getPort("S3"); // get the port for the light (color sensor)
	public static final SensorModes myColor = new EV3ColorSensor(portColor); // create the color sensor object;
	public static final SampleProvider myColorSample = myColor.getMode("RGB");
	public static final float[] sampleColor = new float[3]; // create an array for the sensor
																				// readings
	public static void main(String[] args) {
		int buttonChoice;		
		do {
		      lcd.clear();
		      lcd.drawString("^ start", 0,1);

		      buttonChoice = Button.waitForAnyPress(); 
		    } while (buttonChoice != Button.ID_UP);
		
		if (buttonChoice ==Button.ID_UP) {
			(new Thread() {
				public void run() {
					sample();				}
			}).start();			
		}		 		 
	}
	
	public static void sample() {
		lcd.clear();
		int counter =0;
		while(counter<1000) {
			myColorSample.fetchSample(sampleColor, 0); 
			float r = sampleColor[0]*1000; 
			float g = sampleColor[1]*1000; 
			float b = sampleColor[2]*1000; 
			System.out.print(r +" ");
			System.out.print(g+" ");
			System.out.println(b+" ");
			counter ++;
		}
		
	}

}
