import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class Labyrinth {
	
	static EV3UltrasonicSensor sensorVorne = new EV3UltrasonicSensor(SensorPort.S2);
	static EV3UltrasonicSensor sensorRechts = new EV3UltrasonicSensor(SensorPort.S3);
	
	static SampleProvider frontSampleProvider = sensorVorne.getDistanceMode ();
	static SampleProvider rightSampleProvider = sensorRechts.getDistanceMode ();
	
	static float[] frontSample = new float[frontSampleProvider.sampleSize()];
	static float[] rightSample = new float[rightSampleProvider.sampleSize()];
	
	static NXTRegulatedMotor motorL = Motor.A;
	static NXTRegulatedMotor motorR = Motor.D;
	
	
	public static void main(String[] args) {

		
		motorR.setSpeed(100);
		motorR.setAcceleration(6000);
		motorL.setSpeed(100);
		motorL.setAcceleration(6000);
		
		while (!Button.ESCAPE.isDown()) {
			
			// 1 sense
			frontSampleProvider.fetchSample(frontSample, 0);
			rightSampleProvider.fetchSample(rightSample, 0);
			
			float cl = -100;
			float cr = -100;
			
			float dr = 0.1f;
			
			float al = 500;
			float ar = -500;
			
			//float bl = 0;
			//float br = 0;
			
			// 2 think + move
			float mL = al*(dr - rightSample[0]) + cl;
			float mR = ar*(dr - rightSample[0]) + cr;
			
			if(rightSample[0] < 0.1)
			{
				motorL.rotate((int)mL, true);
				motorR.rotate((int)mR, true);
			}
			
//			if ( (frontSample[0] <= 0.05) && (rightSample[0] < 0.3)) {
//				motorL.rotate(180);
//				motorR.rotate(-190);
//			}
//			else {
//				motorL.rotate((int)((0.1-rightSample[0]) *500), true);
//				motorR.rotate((int)((0.1-rightSample[0]) *-500), true);
//			}
			
			
			// 3 debug
			LCD.drawString("fS = " + frontSample[0], 0, 1);
			LCD.drawString("rS = " + rightSample[0], 0, 2);
			
			LCD.drawString("mL = " + mL, 0, 4);
			LCD.drawString("mR = " + mR, 0, 5);
		}
	}
}
