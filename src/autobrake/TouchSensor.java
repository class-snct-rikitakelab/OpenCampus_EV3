package autobrake;

import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;

public class TouchSensor {

	private EV3TouchSensor touch;
    private SensorMode touchMode;
    private float[] sampleTouch;

    public TouchSensor(EV3TouchSensor touch){
    	this.touch = touch;
        touchMode = this.touch.getTouchMode();
        sampleTouch = new float[touchMode.sampleSize()];
    }

    public boolean isButtonPressed(){
    	touchMode.fetchSample(sampleTouch, 0);
        return ((int)sampleTouch[0] != 0);
    }

}
