package sample;
import java.net.MalformedURLException;
import java.rmi.NotBoundException;
import java.rmi.RemoteException;

import lejos.hardware.Sound;
import lejos.remote.ev3.RMIRegulatedMotor;
import lejos.remote.ev3.RemoteEV3;
import lejos.utility.Delay;

public class RemoteTest {

    public static void main(String[] args) {
        // TODO Auto-generated method stub
        RemoteEV3 ev3 = null;
        RMIRegulatedMotor m = null;

        try {
            //EV3に接続、引数にEV3のIPアドレス
            ev3 = new RemoteEV3("10.0.1.5");
        } catch (RemoteException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (MalformedURLException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (NotBoundException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        //初期化
        ev3.setDefault();
        //モーターのインスタンス作成(ポートAにLargeモーターを接続する)
        m = ev3.createRegulatedMotor("C",'M');
        //ビープ音を鳴らす
        Sound.beep();
        //モーターを180度回転
        try {
            m.resetTachoCount();
            m.rotateTo(0);

            for ( int k = 1; k <= 5; k++) {
                m.setSpeed(200 * k);
                m.forward();
                Delay.msDelay(5000);
            }
            m.stop(true);
        } catch (RemoteException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

}