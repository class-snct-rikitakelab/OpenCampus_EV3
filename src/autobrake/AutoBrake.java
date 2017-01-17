package autobrake;

import java.util.Timer;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.port.TachoMotorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.utility.Delay;

public class AutoBrake {

	static float BLACK = 0.0F;
	static float WHITE = 0.0F;
	static float ALERT_DIS = 0.1F;

	public static void main(String[] args) {
		// TODO 自動生成されたメソッド・スタブ
		TachoMotorPort motorPortL = MotorPort.C.open(TachoMotorPort.class); // 左モータ
	    TachoMotorPort motorPortR = MotorPort.B.open(TachoMotorPort.class); // 右モータ

	    TouchSensor touch = new TouchSensor(new EV3TouchSensor(SensorPort.S1));
		WheelMotor wheel = new WheelMotor(motorPortR,motorPortL);
		BrightSensor bright = new BrightSensor(new EV3ColorSensor(SensorPort.S3));
		Sonar sonar = new Sonar(new EV3UltrasonicSensor(SensorPort.S4));

		TurnCalc turn = new TurnCalc(bright,50.0F,300.0F);

		Alerm alerm = new Alerm();

        wheel.controlWheel(0, 0);
        wheel.resetEncord();

        Sound.setVolume(10);

        Sound.beep();
        calibration(bright, touch);
        turn.setTarget(WHITE, BLACK);

        // リモート接続
        Timer rcTimer = new Timer();
        CommandTask rcTask = new CommandTask(touch);

        // 走行タスク
        Timer driveTimer = new Timer();
        DriveTask driveTask = new DriveTask(turn, wheel);

        rcTimer.scheduleAtFixedRate(rcTask, 0, 20);

        LCD.clear();
        LCD.drawString("READY TO START", 0, 0);

        Sound.beep();
        while(true){
        	if(rcTask.checkStartFlag() == true){
            	break;
            }else{
            }
        	Delay.msDelay(20);
        }


        driveTimer.scheduleAtFixedRate(driveTask, 0, 4);

        Delay.msDelay(3000);

        Sound.setVolume(100);
        while(true){

        	if(sonar.getDisM() <= ALERT_DIS && sonar.getDisM() >= 0.0F){
        		driveTask.stopDrive();
        		alerm.ringAlerm();
        	}else{
        		driveTask.startDrive();
        		alerm.resetAlerm();
        	}

        	if(rcTask.checkStopFlag() == true){
        		break;
        	}

        	Delay.msDelay(20);
        }

        Sound.beep();

        LCD.clear();
        LCD.drawString("END", 0, 0);

        driveTimer.cancel();
        rcTimer.cancel();
        for(int i= 0;i<100;i++){
        	wheel.controlWheel(0, 0);
        }

        Button.waitForAnyEvent();

	}

	//黒と白と階段の輝度値を取得して記録しておく
	private static void calibration(BrightSensor bright,TouchSensor touch){

		boolean flag = false;

		LCD.drawString("Detect BLACK", 0, 0);
		while(true){
			if(touch.isButtonPressed() == true){
				flag = true;
			}else{
				if(flag == true){
					break;
				}
			}
			Delay.msDelay(100);
		}
		BLACK = bright.getBright();
		LCD.clear();
		Sound.beep();
		flag = false;

		LCD.drawString("Detect WHITE", 0, 0);
		while(true){
			if(touch.isButtonPressed() == true){
				flag =true;
			}else{
				if(flag == true){
					break;
				}
			}
			Delay.msDelay(100);
		}
		WHITE = bright.getBright();
		LCD.clear();
		Sound.beep();
		flag = false;
	}

}
