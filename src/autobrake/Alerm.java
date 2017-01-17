package autobrake;

import lejos.hardware.Sound;

public class Alerm {

	private boolean flag = false;

	public Alerm(){
	}

	public void ringAlerm(){
		if(flag == false){
			Sound.systemSound(false, 3);
			flag = true;
		}
	}

	public void resetAlerm(){
		flag = false;
	}

}
