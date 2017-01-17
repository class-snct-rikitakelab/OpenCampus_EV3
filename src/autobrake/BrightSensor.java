package autobrake;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

public class BrightSensor {

	private float min = 0.0F;
	private float max = 1.0F;

	public float min_api = 0.0F;
	public float max_api = 1.0F;

    private EV3ColorSensor colorSensor;
    private SensorMode redMode;           // 輝度検出モード
    private float[] sampleLight;

	public BrightSensor(EV3ColorSensor color){
		colorSensor = color;
		redMode = colorSensor.getRedMode();
		this.sampleLight = new float[this.redMode.sampleSize()];

	}

	public float getBright() {
		this.redMode.fetchSample(sampleLight, 0);
		return ((this.sampleLight[0] - this.min_api)/(this.max_api-this.min_api))*(min-max);
	}

}
