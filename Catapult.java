package classes2;

import lejos.hardware.BrickFinder;
import lejos.robotics.RegulatedMotor;
import lejos.remote.ev3.RemoteRequestEV3;


public class Catapult {
	
	private RegulatedMotor rightMotor;
	private RegulatedMotor leftMotor;
	private RegulatedMotor centerMotor;
	
	private int currentPos = 0;
	
	float [] sample;
			
	public Catapult(){
		
		 RemoteRequestEV3 slaveBrick = null;
	     
	     try{
	         String address = BrickFinder.discover()[0].getIPAddress();
	         slaveBrick = new RemoteRequestEV3(address);
	         System.out.println("The brick is connected");
	          
	     }catch(Exception e){
	         System.out.println("Error has occured. Could not connect the brick");
	     }
	     
	     rightMotor = slaveBrick.createRegulatedMotor("C", 'L');
	     leftMotor = slaveBrick.createRegulatedMotor("A", 'L');
	     centerMotor = slaveBrick.createRegulatedMotor("B", 'L');
	     
	  
	}
	
	public void launch (){
		
		leftMotor.setAcceleration(20000);
		centerMotor.setAcceleration(20000);
		rightMotor.setAcceleration(20000);
		
		
		leftMotor.setSpeed(20000);
		centerMotor.setSpeed(20000);
		rightMotor.setSpeed(20000);
			
		leftMotor.rotate(-140, true);
		centerMotor.rotate(-140, true);
		rightMotor.rotate(-140, false);
		currentPos = 90;
	}
	
	public void pick (){
		leftMotor.setAcceleration(500);
		centerMotor.setAcceleration(500);
		rightMotor.setAcceleration(500);
		
		leftMotor.setSpeed(500);
		centerMotor.setSpeed(500);
		rightMotor.setSpeed(500);
		
		leftMotor.rotate(50, true);
		centerMotor.rotate(50, true);
		rightMotor.rotate(50, false);
		
		leftMotor.setAcceleration(20);
		centerMotor.setAcceleration(20);
		rightMotor.setAcceleration(20);
		
		leftMotor.setSpeed(50);
		centerMotor.setSpeed(50);
		rightMotor.setSpeed(50);
		
		leftMotor.rotate(75, true);
		centerMotor.rotate(75, true);
		rightMotor.rotate(75, false);
		
		currentPos=0;
	}

	public void travelMode() {
		// TODO Auto-generated method stub
		leftMotor.setAcceleration(200);
		centerMotor.setAcceleration(200);
		rightMotor.setAcceleration(200);
		
		
		leftMotor.setSpeed(100);
		centerMotor.setSpeed(100);
		rightMotor.setSpeed(100);
			
		leftMotor.rotate(-90, true);
		centerMotor.rotate(-90, true);
		rightMotor.rotate(-90, false);
		currentPos = 90;
	}
	public void stop(){
		leftMotor.close();
		centerMotor.close();
		rightMotor.close();
	}
}