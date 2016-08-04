/*
 *  EV3waySample.java (for leJOS EV3)
 *  Created on: 2015/05/09
 *  Author: INACHI Minoru
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 */
package linetrace;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Timer;
import java.util.TimerTask;

import balancer.Balancer;
import lejos.hardware.Battery;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.BasicMotorPort;
import lejos.utility.Delay;

/**
 * 2輪倒立振子ライントレースロボットの leJOS EV3 用 Java サンプルプログラム。
 */
public class EV3waySample {
    // 下記のパラメータはセンサ個体/環境に合わせてチューニングする必要があります
    private static final float GYRO_OFFSET          = 0.0F; //ジャイロセンサオフセット値
    private static final float LIGHT_WHITE          = 0.4F; // 白色のカラーセンサ輝度値
    private static final float LIGHT_BLACK          = 0.0F; // 黒色のカラーセンサ輝度値
    private static final float SONAR_ALERT_DISTANCE = 0.3F; // 超音波センサによる障害物検知距離[m]
    private static final int   TAIL_ANGLE_STAND_UP  = 90;   // 完全停止時の角度[度]
    private static final int   TAIL_ANGLE_DRIVE     = 0;    // バランス走行時の角度[度]
    private static final float P_GAIN               = 2.5F; // 完全停止用モータ制御比例係数
    private static final int   PWM_ABS_MAX          = 60;   // 完全停止用モータ制御PWM絶対最大値
    private static final int   SOCKET_PORT          = 7360; // PCと接続するポート
    private static final int   REMOTE_COMMAND_START = 71;   // 'g'
    private static final int   REMOTE_COMMAND_STOP  = 83;   // 's'
    private static final float THRESHOLD = (LIGHT_WHITE+LIGHT_BLACK)/2.0F;	// ライントレースの閾値

    private static float distance = 0.0F;				//距離

    private static ServerSocket    server = null;
    private static Socket          client = null;
    private static InputStream     inputStream = null;
    private static DataInputStream dataInputStream = null;
    private static DataOutputStream dataOutputStream = null;
    private static int             remoteCommand = 0;

    private static EV3Body         body    = new EV3Body();
    private static int             counter = 0;
    private static boolean         alert   = false;

    private static CheckDistance cDistance = new CheckDistance();
	private static float diff[] = new float[]{0.0f , 0.0f};
	private static float s_diff[] = new float[]{0.0f , 0.0f};
	private static int scount = 0;

    /**
     * メイン
     */
    public static void main(String[] args) {
        LCD.drawString("Please Wait...  ", 0, 4);
        body.gyro.reset();
        body.sonar.enable();
        body.motorPortL.setPWMMode(BasicMotorPort.PWM_BRAKE);
        body.motorPortR.setPWMMode(BasicMotorPort.PWM_BRAKE);
        body.motorPortT.setPWMMode(BasicMotorPort.PWM_BRAKE);

        // Java の初期実行性能が悪く、倒立振子に十分なリアルタイム性が得られない。
        // 走行によく使うメソッドについて、HotSpot がネイティブコードに変換するまで空実行する。
        // HotSpot が起きるデフォルトの実行回数は 1500。
        for (int i=0; i < 1500; i++) {
            body.motorPortL.controlMotor(0, 0);
            body.getBrightness();
            body.getSonarDistance();
            body.getGyroValue();
            Battery.getVoltageMilliVolt();
            Balancer.control(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 8000);
        }
        Delay.msDelay(10000);       // 別スレッドで HotSpot が完了するだろう時間まで待つ。

        body.motorPortL.controlMotor(0, 0);
        body.motorPortR.controlMotor(0, 0);
        body.motorPortT.controlMotor(0, 0);
        body.motorPortL.resetTachoCount();   // 左モータエンコーダリセット
        body.motorPortR.resetTachoCount();   // 右モータエンコーダリセット
        body.motorPortT.resetTachoCount();   // 尻尾モータエンコーダリセット
        Balancer.init();            // 倒立振子制御初期化

        // リモート接続
        Timer rcTimer = new Timer();
        TimerTask rcTask = new TimerTask() {  // リモートコマンドタスク
                @Override
                public void run() {
                    if (server == null) { // 未接続
                        try {
                            server = new ServerSocket(SOCKET_PORT);
                            client = server.accept();
                            inputStream = client.getInputStream();
                            dataInputStream = new DataInputStream(inputStream);
                            dataOutputStream = new DataOutputStream(client.getOutputStream());
                        } catch (IOException ex) {
                            ex.printStackTrace();
                            server = null;
                            dataInputStream = null;
                        }
                    } else {
                        try {
                            if (dataInputStream.available() > 0) {
                                remoteCommand = dataInputStream.readInt();
                            }
                        } catch (IOException ex) {
                        }
                    }
                }
            };
        rcTimer.schedule(rcTask, 0, 20);

        while(true){

            body.motorPortL.controlMotor(0, 0);
            body.motorPortR.controlMotor(0, 0);
            body.motorPortT.controlMotor(0, 0);
            body.motorPortL.resetTachoCount();
            body.motorPortR.resetTachoCount();
            body.motorPortT.resetTachoCount();
            body.gyro.reset();
            remoteCommand = 0;
            Balancer.init();

        // スタート待ち
        LCD.drawString("Touch to START", 0, 4);
        boolean touchPressed = false;
        for (;;) {
            tailControl(body, TAIL_ANGLE_STAND_UP); // 完全停止用角度に制御
            if (body.touchSensorIsPressed()) {
                touchPressed = true;          // タッチセンサが押された
            } else {
                if (touchPressed) break;      // タッチセンサが押された後に放した
            }
            if (checkRemoteCommand(REMOTE_COMMAND_START)) break;  // PC で 'g' キーが押された
            Delay.msDelay(20);
        }

        LCD.drawString("Running       ", 0, 4);
        Timer driveTimer = new Timer();
        TimerTask driveTask = new TimerTask() {

    		float p,i,d;
    		float s_p,s_i,s_d,speed,oldDistance;
    		float integral=0.0f;
    		float DELTA_T = 0.004f; //処理時間(s)
    		float KU = 350;			//限界感度
    	    float PU = 0.4f;		//限界感度の振動周期
    	    float KP[] = {80.0F,80.0F,120.0F,80.0F,180.0F,80.0F,180.0F,80.0F,0.0F};//KU  * 0.6f;	//比例定数
    	    float TI = PU  * 0.5f;	//積分時間
    	    float TD = PU  * 0.125f;//微分時間
    	    float KI = 700;			//積分定数
    	    float KD = 30;//KP * TD;		//微分定数
    	    private CheckDistance cDistance = new CheckDistance();
    	    float start_dis = cDistance.getDistance(body);
    	    int   j=0;
    	    float distance[] = {  0.1F,  2.3F,  2.9F,  4.7F,   5.69F,  6.922F,  7.953F,  8.0F, 80.2F};
    	    float FORWARD[]  = { 20.0F, 150.0F, 40.0F, 150.0F , 40.00F, 150.000F, 40.000F, 50.0F,  0.0F};



                @Override
                public void run() {
                    tailControl(body, TAIL_ANGLE_DRIVE); // バランス走行用角度に制御

                    if (++counter >= 40/4) {      // 約40msごとに障害物検知
                        alert = sonarAlert(body); // 障害物検知
                        counter = 0;
                    }
                    float forward =  0.0F; // 前後進命令
                    float turn    =  0.0F; // 旋回命令
                    if (alert) {           // 障害物を検知したら停止
                        forward = 0.0F;
                        turn = 0.0F;
                    } else {
                        //forward = 40.0F;  // 前進命令
                        if(cDistance.getDistance(body) < start_dis + distance[j]){
                        	forward = FORWARD[j];
                        }else{
                        	j++;
                        	forward = FORWARD[j];
                        }

                        /*
                         * 速度型PID制御
                         *

                        if(scount++ > 10){			//約40msごとに
                        	speed = spMeasure.getSpeed(body);
                        	scount=0;
                        	s_diff[0] = s_diff[1];
                			s_diff[1] = speed - 1.0F;
                			integral += (s_diff[1] + s_diff[0]) / 2.0 * DELTA_T;

                			s_p = 0.0f * s_diff[1];
                			s_i = KI * integral;
                			s_d = KD * (s_diff[1] - s_diff[0]) / DELTA_T;

                			forward += s_p;// + s_d + s_i;
                            if (forward > 50.0f) {
                                forward = 50.0F;  // 右旋回命令
                            } else if(forward < -50.0f){
                                turn = -50.0F; // 左旋回命令
                            }
                        	System.out.println(speed);
                        }*/


                        /*
                         * PID制御
                         */
            			diff[0] = diff[1];
            			diff[1] = -body.getBrightness() + THRESHOLD;
            			integral += (diff[1] + diff[0]) / 2.0 * DELTA_T;

            			p = KP[j] * diff[1];
            			i = KI * integral;
            			d = KD * (diff[1] - diff[0]) / DELTA_T;

            			turn  = p + d;// + i;


            			if(turn > 50.0f)
            				turn = 50.0f;
            			else if(turn < -50.0f)
        				turn = -50.0f;
                    }





                    float gyroNow = -body.getGyroValue();              // ジャイロセンサー値
                    int thetaL = body.motorPortL.getTachoCount();     // 左モータ回転角度
                    int thetaR = body.motorPortR.getTachoCount();     // 右モータ回転角度
                    int battery = Battery.getVoltageMilliVolt();      // バッテリー電圧[mV]
                    Balancer.control (forward, turn, gyroNow, GYRO_OFFSET, thetaL, thetaR, battery); // 倒立振子制御
                    body.motorPortL.controlMotor(Balancer.getPwmL(), 1); // 左モータPWM出力セット
                    body.motorPortR.controlMotor(Balancer.getPwmR(), 1); // 右モータPWM出力セット

                    /*if(scount++ > 10){			//約40msごとに表示
                    	float speed = cSpeed.getSpeed(distance);
                    	scount=0;
                    	try {
							dataOutputStream.writeFloat(speed);
						} catch (IOException e) {
						}
                    }*/

                }
            };
        driveTimer.scheduleAtFixedRate(driveTask, 0, 4);

        for (;;) {
            if (body.touchSensorIsPressed() // タッチセンサが押されたら走行終了
                || checkRemoteCommand(REMOTE_COMMAND_STOP)) { // PC で 's' キー押されたら走行終了
                //rcTimer.cancel();
                driveTimer.cancel();
                body.motorPortL.controlMotor(0, 0);
                body.motorPortR.controlMotor(0, 0);
                body.motorPortT.controlMotor(0, 0);
                break;
            }
            Delay.msDelay(20);
        }


        if (server != null) {
            //try { server.close(); } catch (IOException ex) {}
        }
        tailControl(body,0);

        body.motorPortL.controlMotor(0, 0);
        body.motorPortR.controlMotor(0, 0);
        body.motorPortT.controlMotor(0, 0);
        body.motorPortL.resetTachoCount();
        body.motorPortR.resetTachoCount();
        body.motorPortT.resetTachoCount();
        }
    }

    /*
     * 超音波センサによる障害物検知
     * @return true(障害物あり)/false(障害物無し)
     */
    private static final boolean sonarAlert(EV3Body body) {
        float distance = body.getSonarDistance();
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0)) {
            return true;  // 障害物を検知
        }
        return false;
    }

    /*
     * 走行体完全停止用モータの角度制御
     * @param angle モータ目標角度[度]
     */
    private static final void tailControl(EV3Body body, int angle) {
        float pwm = (float)(angle - body.motorPortT.getTachoCount()) * P_GAIN; // 比例制御
        // PWM出力飽和処理
        if (pwm > PWM_ABS_MAX) {
            pwm = PWM_ABS_MAX;
        } else if (pwm < -PWM_ABS_MAX) {
            pwm = -PWM_ABS_MAX;
        }
        body.motorPortT.controlMotor((int)pwm, 1);
    }

    /*
     * リモートコマンドのチェック
     */
    private static final boolean checkRemoteCommand(int command) {
        if (remoteCommand > 0) {
            if (remoteCommand == command) {
                return true;
            }
        }
        return false;
    }
}
