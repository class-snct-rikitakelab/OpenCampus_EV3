package autobrake;

import java.util.TimerTask;

public class DriveTask extends TimerTask {

	TurnCalc turncalc;
	WheelMotor wheel;

	private boolean stopFlag = false;

	DriveTask(TurnCalc turn, WheelMotor wheel){
		turncalc = turn;
		this.wheel = wheel;
	}

	@Override
	public void run() {
		// TODO 自動生成されたメソッド・スタブ

		float turn = turncalc.getTurn();
		float forward = 30.0F;

		float pwmL = forward - turn;
		float pwmR = forward + turn;

		if(this.stopFlag == false){
			wheel.controlWheel((int)pwmR, (int)pwmL);
		}else{
			wheel.controlWheel(0, 0);
		}
	}

	public void stopDrive(){
		stopFlag = true;
	}

	public void startDrive(){
		stopFlag = false;
	}

}
