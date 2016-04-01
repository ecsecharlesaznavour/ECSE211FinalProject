package classes2;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class DualOdometryCorrection extends Thread{

	private EV3ColorSensor rightSensor, leftSensor;
	private SampleProvider Csp1, Csp2;
	private float[] datas1, datas2;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private boolean black1, black2, allowAng, enabled = false;
	private Odometer odometer;
	private final static TextLCD t = LocalEV3.get().getTextLCD();
	
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
		this.datas1 = new float[Csp1.sampleSize()];
		this.datas2 = new float[Csp2.sampleSize()];
	}
	public void run()
	{
		//TODO fill the run method
	}
	
	public void doCorrection()
	{
		//TODO fill the correction code
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
			if(Math.abs(last-newDist) > 15)
			{
				while(Filter < 15 && Math.abs(last-newDist) > 15)
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
