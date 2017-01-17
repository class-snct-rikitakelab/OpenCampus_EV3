package autobrake;

import java.util.TimerTask;

public class CommandTask extends TimerTask{

	TouchSensor touch;

	boolean startFlag = false;
	long startTime = 0;
	boolean stopFlag = false;
	public boolean measureFlag = false;

	CommandTask(TouchSensor touch){
		this.touch = touch;
	}

	@Override
	public void run() {
		// TODO 自動生成されたメソッド・スタブ

		if(touch.isButtonPressed() == true && startFlag == false){
			startFlag=true;
		}else{
			startFlag = false;
		}

		if(touch.isButtonPressed() == true && stopFlag == false){
			stopFlag = true;
		}else{
			stopFlag = false;
		}

	}

	public boolean checkStartFlag(){
		return this.startFlag;
	}

	public boolean checkStopFlag(){
		return this.stopFlag;
	}

}
