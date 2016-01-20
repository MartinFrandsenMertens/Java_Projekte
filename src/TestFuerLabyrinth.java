import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;

public class TestFuerLabyrinth {
	static EV3UltrasonicSensor sensorVorne = new EV3UltrasonicSensor(SensorPort.S2);
	static EV3UltrasonicSensor sensorRechts = new EV3UltrasonicSensor(SensorPort.S3);
	
	static SampleProvider frontSampleProvider = sensorVorne.getDistanceMode ();
	static SampleProvider rightSampleProvider = sensorRechts.getDistanceMode ();
	
	static float[] frontSample = new float[frontSampleProvider.sampleSize()];
	static float[] rightSample = new float[rightSampleProvider.sampleSize()];
	
	static NXTRegulatedMotor motorV = Motor.A;
	static NXTRegulatedMotor motorR = Motor.D;
	
	public static void main(String[] args) {
			
		motorR.setSpeed(100);
		motorR.setAcceleration(6000);
		motorV.setSpeed(100);
		motorV.setAcceleration(6000);	
		
		float distanceRight = Math.min(rightSample[0], 1.0f);
		
		while (!Button.ESCAPE.isDown()) {
			frontSampleProvider.fetchSample(frontSample, 0);

			distanceRight = 0.9f*distanceRight + 0.1f*Math.min(rightSample[0], 1.0f);
			
			int ml = (int)(-50f-(0.1f-distanceRight) *500f + 0.5f);
			int mr = (int)(-50f+(0.1f-distanceRight) *500f + 0.5f);
					
			motorR.rotate(ml, true);
			motorV.rotate(mr, true);
			
			LCD.drawString(distanceRight + "", 0, 3);
			LCD.drawString(ml + "", 0, 5);
			LCD.drawString(mr + "", 0, 6);
//			LCD.drawString(RightSample[0] + "", 0, 5);
			
		}
	}
}