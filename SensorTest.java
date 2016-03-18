package classes;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class SensorTest {
	
	private final static Port port1 = LocalEV3.get().getPort("S1");
	private final static Port port2 = LocalEV3.get().getPort("S2");
	
	public static void main(String[] args)
	{
		@SuppressWarnings("resource")
		EV3UltrasonicSensor usSensor1 = new EV3UltrasonicSensor(port1);
		SampleProvider usDist1 = usSensor1.getMode("Distance");
		float[] data1 = new float[usDist1.sampleSize()];
		
		int option = 0;
		
		while(option != Button.ID_ENTER)
			option = Button.waitForAnyPress();
		
		usSensor1.disable();
		
		@SuppressWarnings("resource")
		EV3UltrasonicSensor usSensor2 = new EV3UltrasonicSensor(port2);
		SampleProvider usDist2 = usSensor2.getMode("Distance");
		float[] data2 = new float[usDist2.sampleSize()];
		
		option = 0;
		
		while(option != Button.ID_ENTER)
			option = Button.waitForAnyPress();
		
		usSensor1.enable();
		
		while(option != Button.ID_ESCAPE)
			option = Button.waitForAnyPress();
		
		System.exit(0);
	}
}
