import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class Ausweichen {
	
	static EV3UltrasonicSensor sensorVorne = new EV3UltrasonicSensor(SensorPort.S2);
	static EV3UltrasonicSensor sensorRechts = new EV3UltrasonicSensor(SensorPort.S3);
	
	static SampleProvider frontSampleProvider = sensorVorne.getDistanceMode ();
	static SampleProvider rightSampleProvider = sensorRechts.getDistanceMode ();
	
	static float[] frontSample = new float[frontSampleProvider.sampleSize()];
	static float[] rightSample = new float[rightSampleProvider.sampleSize()];
	
	static NXTRegulatedMotor motorL = Motor.A;
	static NXTRegulatedMotor motorR = Motor.D;
	
	public static void main(String[] args) {
		
		motorR.setSpeed(200);
		motorR.setAcceleration(6000);
		motorL.setSpeed(200);
		motorL.setAcceleration(6000);
		
		while (!Button.ESCAPE.isDown()) {
			
			frontSampleProvider.fetchSample(frontSample, 0);
			
			if (frontSample[0] > 0.1) {
				motorR.rotate(-90);
				motorL.rotate(-90);
			}
			else {
				motorR.rotate(-90);
				motorL.rotate(90);
			}
		}
	}
}
