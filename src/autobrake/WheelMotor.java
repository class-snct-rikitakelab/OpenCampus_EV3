package autobrake;

import lejos.hardware.port.BasicMotorPort;
import lejos.hardware.port.TachoMotorPort;

public class WheelMotor {

	private TachoMotorPort motorPortL; // 左モータ
    private TachoMotorPort motorPortR; // 右モータ

	public WheelMotor(TachoMotorPort motorR,TachoMotorPort motorL){
		this.motorPortL = motorL;
		this.motorPortR = motorR;

		this.motorPortL.setPWMMode(BasicMotorPort.PWM_BRAKE);
		this.motorPortR.setPWMMode(BasicMotorPort.PWM_BRAKE);

	}

	public void resetEncord(){
		this.motorPortL.resetTachoCount();
		this.motorPortR.resetTachoCount();
	}

	public void controlWheel(int right, int left) {

		this.motorPortL.controlMotor(left, 1);
		this.motorPortR.controlMotor(right, 1);
	}

	public int getEncordL(){
		return this.motorPortL.getTachoCount();
	}

	public int getEncordR(){
		return this.motorPortR.getTachoCount();
	}

}
