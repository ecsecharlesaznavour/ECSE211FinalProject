import lejos.hardware.BrickFinder;
import lejos.robotics.RegulatedMotor;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.remote.ev3.RemoteRequestEV3;


public class Catapult {
	
	static RegulatedMotor rightMotor;
	static RegulatedMotor leftMotor;
	static RegulatedMotor centerMotor;
	
	static int currentPos = 0;
	
	
	public static void main (String[]args){
		
		 RemoteRequestEV3 slaveBrick = null;
	     
	     try{
	         String address = BrickFinder.discover()[0].getIPAddress();
	         //masterBrick = new RemoteRequestEV3(address);
	         slaveBrick = new RemoteRequestEV3(address);
	         System.out.println("The brick is connected");
	          
	     }catch(Exception e){
	         System.out.println("Error has occured. Could not connect the brick");
	     }
	     
	     rightMotor = slaveBrick.createRegulatedMotor("C", 'L');
	     leftMotor = slaveBrick.createRegulatedMotor("A", 'L');
	     centerMotor = slaveBrick.createRegulatedMotor("B", 'L');
	     
	  
		
		
		
		
		 while (true){
	            int buttonPress = 0;
	            buttonPress = Button.waitForAnyPress();
	            if (buttonPress == Button.ID_DOWN){
	            	if (currentPos!= 0){
	            		try{
	            			pick();
	            		}
	            		catch(Exception e){
	            			System.out.println("pick problem");
	            		}
	            	}
	            }
	            if(buttonPress == Button.ID_UP){
	            	try{
	            		launch();
	            	}
	            	catch(Exception e){
	            			System.out.println("launch problem");
	            	}
	            }
	            if (buttonPress == Button.ID_ESCAPE){
	            	leftMotor.close();
	            	rightMotor.close();
	            	centerMotor.close();
	            	break;
	            }
	     }
	}
	
	public static void launch (){
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
	
	public static void pick (){
		leftMotor.setAcceleration(500);
		centerMotor.setAcceleration(500);
		rightMotor.setAcceleration(500);
		
		leftMotor.setSpeed(500);
		centerMotor.setSpeed(500);
		rightMotor.setSpeed(500);
		
		leftMotor.rotate(70, true);
		centerMotor.rotate(70, true);
		rightMotor.rotate(70, false);
		
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
	
}
