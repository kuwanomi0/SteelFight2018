package sample;

import lejos.hardware.motor.Motor;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class MotorSample {

    static RegulatedMotor testMotor = Motor.C;

    public static void main(String[] args) {
        testMotor.resetTachoCount();
        testMotor.rotateTo(0);

        for ( int k = 1; k <= 5; k++) {
            testMotor.setSpeed(200 * k);
            testMotor.forward();
            Delay.msDelay(5000);
        }
        /*
        for ( int k = 1; k <= 5; k ++) {
            testMotor.setSpeed(100 * k);
            testMotor.backward();
            Delay.msDelay(2000);
        }
        */
    }
}
