package autobrake;

import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class Sonar {

	private EV3UltrasonicSensor sonar;
	 private SampleProvider distanceMode;  // 距離検出モード
	 private float[] sampleDistance;


	public Sonar(EV3UltrasonicSensor sonar){
		this.sonar = sonar;

		distanceMode = this.sonar.getDistanceMode(); // 距離検出モード
        sampleDistance = new float[distanceMode.sampleSize()];
        this.sonar.enable();
	}

	public float getDisCM(){
		this.distanceMode.fetchSample(sampleDistance, 0);
		return this.sampleDistance[0] * 100;
	}

	public float getDisM(){
		this.distanceMode.fetchSample(sampleDistance, 0);
		return this.sampleDistance[0];
	}

}
