import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
//import lejos.utility.Delay;
import lejos.hardware.Button;

public class AbstandsFahren {
	static EV3UltrasonicSensor sensorVorne = new EV3UltrasonicSensor(SensorPort.S2);
	static EV3UltrasonicSensor sensorRechts = new EV3UltrasonicSensor(SensorPort.S3);
	
	static SampleProvider frontSampleProvider = sensorVorne.getDistanceMode ();
	static SampleProvider rightSampleProvider = sensorRechts.getDistanceMode ();
	
	static float[] frontSample = new float[frontSampleProvider.sampleSize()];
	static float[] rightSample = new float[rightSampleProvider.sampleSize()];
	
	static NXTRegulatedMotor motorV = Motor.A;
	static NXTRegulatedMotor motorR = Motor.D;
	
	public static void main(String[] args) {
			
		motorR.setSpeed(150);
		motorR.setAcceleration(6000);
		motorV.setSpeed(150);
		motorV.setAcceleration(6000);	
		
		while (!Button.ESCAPE.isDown()) {
			frontSampleProvider.fetchSample(frontSample, 0);
			rightSampleProvider.fetchSample(rightSample, 0);
			
			
			int mr = ((int)((0.1-frontSample[0]) *500));
			int ml = ((int)((0.1-frontSample[0]) *500));
					
			motorR.rotate(ml, true);
			motorV.rotate(mr, true);
					
		}
	}
}