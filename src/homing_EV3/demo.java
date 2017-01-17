package homing_EV3;

import java.io.DataInputStream;
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
 * Java sample program for leJOS EV3 of two-wheel inverted pendulum line trace robot.
 */
public class demo {
    // The following parameters will need to be tuned to suit the individual sensor / environment
    private static final float GYRO_OFFSET          = 0.0F; //gyro sensor offset value
    private static final float LIGHT_WHITE          = 0.3F; // white color sensor brightness value

    private static final float LIGHT_BLACK          = 0.0F; //  black color sensor brightness value
    private static final float SONAR_ALERT_DISTANCE = 0.3F; // obstacle detection distance by ultrasonic sensor [m]
    private static final int   TAIL_ANGLE_STAND_UP  = 94;   //  complete stop when the angle [degrees]
    private static final int   TAIL_ANGLE_DRIVE     = 3;    // balance during running of the angle [degrees]
    private static final float P_GAIN               = 2.5F; //  complete stop motor control proportionality coefficient
    private static final int   PWM_ABS_MAX          = 60;   // complete stop motor control PWM absolute maximum
    private static final int   SOCKET_PORT          = 7360; // port to be connected to the PC
    private static final int   REMOTE_COMMAND_START = 71;   // 'g'
    private static final int   REMOTE_COMMAND_STOP  = 83;   // 's'
    private static final float THRESHOLD = (LIGHT_WHITE+LIGHT_BLACK)/2.0F;	// line threshold of trace
    private static final int		CNTLIMIT = 95;
    private static final float 	KP = 200.0F;

    private static ServerSocket    server = null;
    private static Socket          client = null;
    private static InputStream     inputStream = null;
    private static DataInputStream dataInputStream = null;
    private static int             remoteCommand = 0;

    private static EV3Body         body    = new EV3Body();
    private static int             counter = 0;
    private static boolean         alert   = false; //Detect some range before found obstacles
	private static boolean         aware   = false; //Detect an obstacles
	private static boolean         isRight = true;
	private static boolean         isFront = false;
	private static int			turncounter = 0;
	private static int 			stopcounter = 0;
	private static int 			uscounter = 0;
	private static int 			seekrange = 0;
	private static float 		lastdistance = 0;


    /**
     *  Main
     */
    public static void main(String[] args) {

        LCD.drawString("Please Wait...  ", 0, 4);
        body.gyro.reset();
        body.sonar.enable();
        body.motorPortL.setPWMMode(BasicMotorPort.PWM_BRAKE);
        body.motorPortR.setPWMMode(BasicMotorPort.PWM_BRAKE);
        body.motorPortT.setPWMMode(BasicMotorPort.PWM_BRAKE);


        // Initial execution performance of Java is bad, it is not possible to obtain sufficient real-time to an inverted pendulum.
        // About the methods frequently used to traveling, to empty run until HotSpot is converted to native code.
        // Default number of executions that HotSpot occurs 1500.
        for (int i=0; i < 1500; i++) {
            body.motorPortL.controlMotor(0, 0);
            body.getBrightness();
            body.getSonarDistance();
            body.getGyroValue();
            Battery.getVoltageMilliVolt();
            Balancer.control(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 8000);
        }
        Delay.msDelay(1000);       // In another thread wait until the wonder time that HotSpot is complete.

        body.motorPortL.controlMotor(0, 0);
        body.motorPortR.controlMotor(0, 0);
        body.motorPortT.controlMotor(0, 0);
        body.motorPortL.resetTachoCount();   // left motor encoder reset
        body.motorPortR.resetTachoCount();   // right motor encoder reset
        body.motorPortT.resetTachoCount();   // tail motor encoder reset
        Balancer.init();            // inverted pendulum control initialization


        // Remote connection
        Timer rcTimer = new Timer();
        TimerTask rcTask = new TimerTask() {  //  remote command task
                @Override
                public void run() {
                    if (server == null) { // Not connected
                        try {
                            server = new ServerSocket(SOCKET_PORT);
                            client = server.accept();
                            inputStream = client.getInputStream();
                            dataInputStream = new DataInputStream(inputStream);
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

        // Waiting for the start of
        LCD.drawString("Touch to START", 0, 4);
        boolean touchPressed = false;
        for (;;) {
            tailControl(body, TAIL_ANGLE_STAND_UP); //complete stop for angle control
            if (body.touchSensorIsPressed()) {
                touchPressed = true;          // touch sensor is pressed
            } else {
                if (touchPressed) break;      // touch sensor I was released after being pressed
            }
            if (checkRemoteCommand(REMOTE_COMMAND_START)) break;  // in the PC 'g' key is pressed
            Delay.msDelay(20);
        }


        LCD.drawString("Running       ", 0, 4);
        Timer driveTimer = new Timer();
        TimerTask driveTask = new TimerTask() {

                @Override
                public void run() {
                    //tailControl(body, TAIL_ANGLE_DRIVE); // balance for running angle to control

                    if (++counter >= 40/4) {      // for each // about 40ms and obstacle detection
                        alert = sonarAlert(body); // obstacle detection
                        aware = sonarAware(body); // detection before found obstacle
                        counter = 0;
                    }

                    float forward =  0.0F; // forward-reverse instruction
                    float turn    =  0.0F; // turning instruction
                    float distance = body.getSonarDistance();
                    if(body.touchSensorIsPressed())stopcounter=300;
                    if(distance < 0.50f)uscounter = 400;

                    if(stopcounter >= 0){//リセット後安定させるために少し停止(forword,turnの初期値を使う)
                    	stopcounter--;
                    }
                    else if(uscounter <= 0){
                    	if(isRight^isFront){//超音波センサーが感知していない時は回る
                    		turn = 40.0F;
                    	}
                    	else{
                    		turn = -40.0F;
                    	}
                    	if(++turncounter - seekrange * 50 > 250){
                    		if(isFront){
                    			isRight = !isRight;++seekrange;
                    		}
                    		isFront = !isFront;
                    		turncounter = 0;
                    	}

                    }
                    else{//超音波センサーが感知してる時とその直後は前に動く



                    	if(distance < 0.50F){
                    		lastdistance = distance;
                    	}

                    	forward = KP*(lastdistance - 0.15F);
                    	if(forward < -40.0F){

                    		forward = -40.0F;
                    	}
                    	else if(forward > 40.0F){
                    		forward =40.0F;
                    	}
                  		if(uscounter <= 200){//前に動いてから回るとき、その前に減速してその後の動作を安定させる
                           	forward *= 1 - uscounter/200;
                        }

                    	isFront = false;
                    	seekrange = 0;
                        --uscounter;
                    }


                    /*if(stopcounter>0){
                    	forward = 0;
                    	turn = 0;
                    	stopcounter--;
                    }*/

                    float gyroNow = -body.getGyroValue();              // gyro sensor value
                    int thetaL = body.motorPortL.getTachoCount();     //  left motor rotation angle
                    int thetaR = body.motorPortR.getTachoCount();     // right motor rotation angle
                    int battery = Battery.getVoltageMilliVolt();      //  battery voltage [mV]
                    if(stopcounter>200){// タッチセンサーが押されている時とその直後は停止してしっぽを立てる
                    	seekrange = 0;
                        body.motorPortL.resetTachoCount();   // left motor encoder reset
                        body.motorPortR.resetTachoCount();   // right motor encoder reset
                        Balancer.init();            // inverted pendulum control initialization
                        Balancer.control (0, 0, gyroNow, GYRO_OFFSET, thetaL, thetaR, battery); // inverted pendulum control
                    	body.motorPortL.controlMotor(0, 1); // left motor PWM output set
                    	body.motorPortR.controlMotor(0, 1); // right motor PWM output set
                    	tailControl(body, TAIL_ANGLE_STAND_UP);
                    }else{
                    	Balancer.control (forward, turn, gyroNow, GYRO_OFFSET, thetaL, thetaR, battery); // inverted pendulum control
                    	body.motorPortL.controlMotor(Balancer.getPwmL(), 1); // left motor PWM output set
                    	body.motorPortR.controlMotor(Balancer.getPwmR(), 1); // right motor PWM output set
                    	tailControl(body, TAIL_ANGLE_DRIVE);
                    }

                }
            };
        driveTimer.scheduleAtFixedRate(driveTask, 0, 4);

        for (;;) {

        	/*if(body.touchSensorIsPressed()){
        		rcTimer.cancel();
        		driveTimer.cancel();
        		Delay.msDelay(3000);
        		rcTimer.schedule(rcTask, 0, 20);
                driveTimer.scheduleAtFixedRate(driveTask, 0, 4);


        	}*/

        }

    }

    /*
     * The obstacle detection by the ultrasonic sensor
     * @return true(there is an obstacle) / false (no obstacle))
     */
    private static final boolean sonarAlert(EV3Body body) {
        float distance = body.getSonarDistance();
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0)) {
            return true;  // obstacle; return true
        }
        return false;
    }

    /*
     * The checking gray color point by sheck point that is not black color
     * @return true(not black color) / false (black color))
     */
    private static final boolean checkGrayzone(EV3Body body){
    	float value = body.getBrightness();
    	if(value >= 0.06 && value<=0.14){
    		return true;
    	}else{
    		return false;
    	}
    }
    private static final boolean checkNotBlackzone(EV3Body body){
    	float value = body.getBrightness();
    	if(value > 0.05){
    		return true;
    	}else{
    		return false;
    	}
    }

    /*
     * The obstacle awareness by the ultrasonic sensor
     * @return true(there is an obstacle) / false (no obstacle))
     */
    private static final boolean sonarAware(EV3Body body) {
        float distance = body.getSonarDistance();
        if ((distance <= SONAR_ALERT_DISTANCE+0.2F) && (distance >= SONAR_ALERT_DISTANCE)) {
            return true;  // obstacle; return true
        }
        return false;
    }

    /*
     *  Traveling body full stop motor of angle control
     * @param angle motor target angle [degree]
     */
    private static final void tailControl(EV3Body body, int angle) {
        float pwm = (float)(angle - body.motorPortT.getTachoCount()) * P_GAIN; // proportional control
        // PWM output saturation processing
        if (pwm > PWM_ABS_MAX) {
            pwm = PWM_ABS_MAX;
        } else if (pwm < -PWM_ABS_MAX) {
            pwm = -PWM_ABS_MAX;
        }
        body.motorPortT.controlMotor((int)pwm, 1);
    }

    /*
     * Check the remote command
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
