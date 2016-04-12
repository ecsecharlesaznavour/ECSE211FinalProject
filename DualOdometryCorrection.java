package classes2;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class DualOdometryCorrection extends Thread{

	private EV3ColorSensor rightSensor, leftSensor;
	private SampleProvider Csp1, Csp2;
	private float[] data;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private Odometer odometer;
	private boolean correct = false;
	private int x,y = -1;
	
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
		int col = -1;
		while(true)
		{
			if(correct)
			{
				int col2;
				if(col==-1)
					col = getCSFilteredData(Csp1, data, 0);
				
				if(col - (col2 = getCSFilteredData(Csp1, data, col)) > 7)
				{
					Adjust();
					col = col2;
					try{ Thread.sleep(1000);} catch(Exception e){}
				}
			}
			else
			{
				col = -1;
			}
		}
	}
	
	private void Adjust()
	{
		if(Math.abs(odometer.getAng()-90) < 10)
			odometer.setY((y++)*30.0 + 12);
		else if(Math.abs(odometer.getAng() - 270) < 10)
			odometer.setY((y--)*30.0 + 18);
		else if(Math.abs(odometer.getAng() - 180) < 10)
			odometer.setX((x--)*30.0 + 18);
		else
			odometer.setX((x++)*30.0 + 12);
	}
	
	public void doCorrection()
	{
		correct = true;
	}
	
	public void stopCorrection()
	{
		correct = false;
	}
	
	public void relocalize(double ang)
	{
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
			if(col1-(col1b = getCSFilteredData(Csp1, data, col1))> 7)
			{
				if(col2-(col2b = getCSFilteredData(Csp2, data, col2)) < 7)
				{
					rightMotor.stop();
					col1 = col1b;
					while(col2-(col2b = getCSFilteredData(Csp2, data, col2))<7)
						col2 = col2b;
					leftMotor.stop();
				}
				break;
			} else if(col2-(col2b = getCSFilteredData(Csp2, data, col2))> 7)
			{
				if(col1-(col1b = getCSFilteredData(Csp1, data, col1))< 7)
				{
					leftMotor.stop();
					col2 = col2b;
					while(col1-(col1b = getCSFilteredData(Csp1, data, col1))<7)
						col1 = col1b;
					rightMotor.stop();
				}
				break;
			}
			col1 = col1b;
			col2 = col2b;
		}
		Adjust();
		
		odometer.setAng(ang);
		leftMotor.setSpeed(300);
		rightMotor.setSpeed(300);
		
		leftMotor.forward();
		rightMotor.forward();
		
		try{Thread.sleep(500);} catch(Exception e) {}
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
			if(Math.abs(last-newDist) > 5)
			{
				while(Filter < 15 && Math.abs(last-newDist) > 5)
				{
					Filter++;
					sp.fetchSample(data, 0);
					newDist = (int) (data[0]*100);
				}
			}
		}
		
		return newDist;
	}

}
