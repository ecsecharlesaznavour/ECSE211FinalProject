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
	
	/**
	 * Default Constructor for the DualOdometryCorrection class.
	 * @param Csp1 SampleProvider for the right ColorSensor.
	 * @param sp2 SampleProvider for the left ColorSensor.
	 * @param leftMotor Left Motor of the robot.
	 * @param rightMotor Right Motor of the robot.
	 * @param odometer Odometer for the robot.
	 */
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
	
	/**
	 * Run method for the DualOdometryCorrection thread. Responsible for handling the data returned by the sensors.
	 */
	public void run()
	{
		int data1 = getCSFilteredData(Csp1, datas1, 0);
		int data2 = getCSFilteredData(Csp2, datas2, 0);
		while(true)
		{
			int data3 = getCSFilteredData(Csp1, datas1, data1);
			int data4 = getCSFilteredData(Csp2, datas2, data2);
			if(Math.abs(data1 - data3) > 15)
				black1 = !black1;
			if(Math.abs(data2 - data4) > 15)
				black2 = !black2;
			
			if(enabled)
			{
				if(black1 && !black2)
				{
					if(allowAng)
					{
						leftMotor.stop(true);
						while(black2 != black1 )
						{
							data2 = data4;
							data4 = getCSFilteredData(Csp2, datas2, data2);
							black2 = ((data2 - data4) > 15);
						}
						leftMotor.forward();
						correctOdo(10);
						try{Thread.sleep(500);}catch(Exception e){}
						allowAng = false;
					}
				} else if(black2 && !black1)
				{
					if(allowAng)
					{
						rightMotor.stop(true);
						while(black1 != black2)
						{
							data1 = data3;
							data3 = getCSFilteredData(Csp1, datas1, data1);
							black1 = ((data1 - data3) > 15);
						}
						rightMotor.forward();
						correctOdo(10);
						try{Thread.sleep(500);}catch(Exception e){}
						allowAng = false;
					}
				}
				
			}
			
			if(black1 || black2)
			{
				correctOdo(10);
				try{Thread.sleep(500);}catch(Exception e){}
			}
			
			t.drawString("" + data3, 0, 3);
			t.drawString("" + data4, 0, 4);
			
			data1 = data3;
			data2 = data4;
			
		}
	}
	
	/**
	 * Corrects the angle and position of the Odometer under a certain threshold.
	 * @param Tthreshold Threshold for the angle to correct.
	 */
	public void correctOdo(double Tthreshold)
	{
		double theta = odometer.getAng();
		double x1 = theta - 90;
		double x2 = theta - 180;
		double x3 = theta - 270;
		if(Math.abs(x1) < Tthreshold)
		{
			odometer.setAng(90);
			float Y = (float) odometer.getY();
			int factor = Math.round(Y/30);
			odometer.setY(factor*30);
		}
		else if(Math.abs(x2) < Tthreshold)
		{
			odometer.setAng(180);
			float X = (float) odometer.getX();
			int factor = Math.round(X/30);
			odometer.setX(factor*30);
		}
		else if(Math.abs(x3) < Tthreshold)
		{
			odometer.setAng(270);
			float Y = (float) odometer.getY();
			int factor = Math.round(Y/30);
			odometer.setY(factor*30);
		}
		else 
		{
			odometer.setAng(0);
			float X = (float) odometer.getX();
			int factor = Math.round(X/30);
			odometer.setX(factor*30);
		}
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
			if(Math.abs(last-newDist) > 0.15)
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
			leftSensor.setFloodlight(true);
			rightSensor.setFloodlight(true);
			enabled = false;
		}
	}
	
	public void setAllow(boolean state)
	{
		synchronized(this)
		{
			this.allowAng = state;
		}
	}

}
