package classes2;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class Demo {
	
	private final static EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private final static EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private final static EV3UltrasonicSensor frontSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
	private final static EV3UltrasonicSensor rightSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));
	private final static EV3ColorSensor RIGHTSensor = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	private final static EV3ColorSensor LEFTSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));

	public static void main(String[] args)
	{
		Stopper stop = new Stopper();
		stop.start();
		
		Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
		DualOdometryCorrection odoCor = new DualOdometryCorrection(LEFTSensor, RIGHTSensor, leftMotor, rightMotor, odo);
		
		MasterBrick MB = new MasterBrick(leftMotor, rightMotor, frontSensor, rightSensor, odo, odoCor);
		
		odoCor.start();
		
		MB.doLocalization();
		MB.turnTo(0);
		MB.travelTo(60, 30);
		
		System.exit(0);
	}
}

class Stopper extends Thread
{
	public void run()
	{
		int option = 0;
		while(option != Button.ID_ESCAPE)
			option = Button.waitForAnyPress();
		
		System.exit(0);
	}
}
