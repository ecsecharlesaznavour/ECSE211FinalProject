package classes2;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class DualOdometryCorrection extends Thread{

	private EV3ColorSensor rightSensor, leftSensor;
	private SampleProvider Csp1, Csp2;
	private float[] data;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private boolean black1, black2, allowAng, enabled = false;
	private Odometer odometer;
	private final static TextLCD t = LocalEV3.get().getTextLCD();
	
	private int lightThreshold = 20;
	
	public DualOdometryCorrection(EV3ColorSensor leftSensor, EV3ColorSensor rightSensor,
			EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			Odometer odometer)
	{
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.rightSensor = rightSensor;
		this.leftSensor = leftSensor;
		this.Csp1 = this.rightSensor.getRedMode();
		this.Csp2 = this.leftSensor.getRedMode();
		this.data = new float[Csp1.sampleSize()];
	}
	public void run()
	{
		while (enabled){
			int result = getCSFilteredData(Csp1, data, 0);
			if (result < lightThreshold){
				Sound.beep();
				if (odometer.getAng()<10 && odometer.getAng()>350){  //if robot moving along y axis
					double currentY = odometer.getY();
				    if (currentY % 30 < 10){                         //round currentY position to nearest multiple of 30
				        odometer.setY(currentY - (currentY%30));
				    }
				    else if (currentY % 30 > 20){
				        odometer.setY(currentY + (30 - currentY%30));
				    }
				}
				
				else if (odometer.getAng()<100 && odometer.getAng()>80){  //if robot moving along x axis
					double currentX = odometer.getX();
				    if (currentX % 30 < 10){                         //round currentX position to nearest multiple of 30
				        odometer.setX(currentX - (currentX%30));
				    }
				    else if (currentX % 30 > 20){
				        odometer.setY(currentX + (30 - currentX%30));
				    }
				}
			}
		}
	}
	
	public void relocalize()
	{
		Enable();
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		
		leftMotor.forward();
		rightMotor.forward();
		
		int col1 = getCSFilteredData(Csp1, data, 0);
		int col2 = getCSFilteredData(Csp2, data, 0);
		
		int col1b;
		int col2b;
		while(true)
		{
			if(Math.abs((col1b = getCSFilteredData(Csp1, data, col1))-col1)> 10)
			{
				rightMotor.stop();
				col1 = col1b;
				while(Math.abs((col2b = getCSFilteredData(Csp2, data, col2))-col2)> 10);
				rightMotor.forward();
				break;
			} else if(Math.abs((col2b = getCSFilteredData(Csp2, data, col2))-col2)> 10)
			{
				leftMotor.stop();
				col2 = col2b;
				while(Math.abs((col2b = getCSFilteredData(Csp1, data, col1))-col1)> 10);
				leftMotor.forward();
				break;
			}
		}
		
		Disable();
	}
	
	/**
	 * Filters the data returned by the desired Sensor.
	 * @param sp SampleProvider of the Sensor.
	 * @param datas Data array of the Sensor.
	 * @param lData Last Data of the Sensor.
	 * @return Filtered Data detected by the Sensor.
	 */
	private int getCSFilteredData(SampleProvider sp, float[] data, int last)
	{
		sp.fetchSample(data, 0);
		int newDist = (int) (data[0]*100);
		int Filter = 0;
		
		if(last == 0)
			return newDist;
		else
		{
			if(Math.abs(last-newDist) > 10)
			{
				while(Filter < 15 && Math.abs(last-newDist) > 10)
				{
					Filter++;
					sp.fetchSample(data, 0);
					newDist = (int) (data[0]*100);
				}
			}
		}
		
		return newDist;
	}
	
	public void Enable()
	{
		synchronized(this)
		{
			leftSensor.setFloodlight(true);
			rightSensor.setFloodlight(true);
			enabled = true;
		}
	}
	
	public void Disable()
	{
		synchronized(this)
		{
			leftSensor.setFloodlight(false);
			rightSensor.setFloodlight(false);
			enabled = false;
		}
	}

}
