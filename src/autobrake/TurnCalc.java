package autobrake;

public class TurnCalc {

	private float KP = 0.0F;
	private float KD = 0.0F;

	private float BrightTarget = 0.0F;

	private BrightSensor bright;

	private float prediff = 0.0F;

	public TurnCalc(BrightSensor bright, float KP, float KD){
		this.bright = bright;
		this.KP = KP;
		this.KD = KD;
	}

	public float getTurn() {

		float curdiff = BrightTarget - bright.getBright();

		float proportion = KP * curdiff;

		float differential = KD * (curdiff - prediff);

		prediff = curdiff;

		return (int)(proportion) * -1;
	}

	public void setTarget(float white, float black) {
		this.BrightTarget = (white+black)/2.0F;
	}

}
