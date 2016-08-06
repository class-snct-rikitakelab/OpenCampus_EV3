package linetrace;

public class CheckDistance{

	private float distance;

	public float getDistance(EV3Body body){

		int distL = body.motorPortL.getTachoCount();
		int distR = body.motorPortR.getTachoCount();

		distance = (distL + distR) / 2;

		//distance = (float)((distR + distL) / 18 * Math.PI);	//車輪半径  約 20 mm

		return distance/360.0F * 0.262F;
	}

}
