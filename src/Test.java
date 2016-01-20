import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class Test {
	
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
		
		float distanceFront = Math.min(frontSample[0], 1.0f);
		
		while (!Button.ESCAPE.isDown()) {
			frontSampleProvider.fetchSample(frontSample, 0);
			rightSampleProvider.fetchSample(rightSample, 0);
			
			distanceFront = 0.01f*distanceFront + 0.99f*Math.min(frontSample[0], 1.0f);
			
			motorL.rotate((int)((0.1-frontSample[0]) *500), true);
			motorR.rotate((int)((0.1-frontSample[0]) *500), true);	
		}
	}
}
