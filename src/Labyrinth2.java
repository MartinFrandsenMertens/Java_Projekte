import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class Labyrinth2 {
	
	static EV3UltrasonicSensor sensorVorne = new EV3UltrasonicSensor(SensorPort.S2);
	static EV3UltrasonicSensor sensorRechts = new EV3UltrasonicSensor(SensorPort.S3);
	
	static SampleProvider frontSampleProvider = sensorVorne.getDistanceMode ();
	static SampleProvider rightSampleProvider = sensorRechts.getDistanceMode ();
	
	static float[] frontSample = new float[frontSampleProvider.sampleSize()];
	static float[] rightSample = new float[rightSampleProvider.sampleSize()];
	
	static NXTRegulatedMotor motorL = Motor.A;
	static NXTRegulatedMotor motorR = Motor.D;
	
	
	public static void main(String[] args) {

		
		motorR.setSpeed(300);
		motorR.setAcceleration(6000);
		motorL.setSpeed(300);
		motorL.setAcceleration(6000);
		
		while (!Button.ESCAPE.isDown()) {
			// 1sense
			frontSampleProvider.fetchSample(frontSample, 0);
			rightSampleProvider.fetchSample(rightSample, 0);
			
			// 2 think + move
			if ((frontSample[0] <= 0.05) && (rightSample[0] < 0.3)) {
				motorL.rotate(180);
				motorR.rotate(-190);
				}
			else {
				rightSampleProvider.fetchSample(rightSample, 0);
					
				if (rightSample[0] < 0.1) {
					motorL.rotate (-30);
					motorR.rotate (-60);
					}
				else {
					motorL.rotate (-60);
					motorR.rotate (-30);
				}
			}
		}
	}
}