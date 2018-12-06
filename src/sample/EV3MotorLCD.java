package sample;

import lejos.hardware.Button;
import lejos.hardware.Key;
import lejos.hardware.KeyListener;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.RegulatedMotorListener;

public class EV3MotorLCD {

    private static RegulatedMotor m;

    public static void main(String[] args) {
        m = new EV3LargeRegulatedMotor(MotorPort.C);
        m.addListener(new RotateStateEvent()); //モーター回転状態イベントの登録
        Button.UP.addKeyListener(new ButtonEvent(m)); //ボタン押下状態イベントの登録
        Button.DOWN.addKeyListener(new ButtonEvent(m));
        Button.ESCAPE.addKeyListener(new ButtonEvent(m));
        while ( true ){}
    }
}

class RotateStateEvent implements RegulatedMotorListener{ //モーター回転状態変化イベント
    @Override
    public void rotationStarted(RegulatedMotor motor, int tachoCount, boolean stalled, long timeStamp) { //回転開始時のイベント
        System.out.println("ROTATE");
    }

    @Override
    public void rotationStopped(RegulatedMotor motor, int tachoCount, boolean stalled, long timeStamp) { //回転停止時のイベント
        System.out.println("STOP");
    }

}

class ButtonEvent implements KeyListener{ //ボタン押下状態イベント
    private RegulatedMotor _Motor;

    public ButtonEvent(RegulatedMotor Motor) {
        _Motor = Motor;
    }

    @Override
    public void keyPressed(Key k) { //モーターを押した時のイベント
        switch (k.getId()){
        case Button.ID_UP:
             System.out.println("UP");
             _Motor.forward();
             break;
        case Button.ID_DOWN:
             System.out.println("DOWN");
             _Motor.backward();
             break;
        }
    }

    @Override
    public void keyReleased(Key k) { //ボタンを離した時のイベント
        switch (k.getId()){
        case Button.ID_UP:
             _Motor.stop();
             break;
        case Button.ID_DOWN:
             _Motor.stop();
             break;
        case Button.ID_ESCAPE:
            System.out.println("ESCAPE");
            _Motor.stop();
            System.exit(0);
            break;
        }
    }

}