package sample;

import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;

public class DoubleSensorLineTraceSample {
    static RegulatedMotor leftMotor   = Motor.B;
    static RegulatedMotor rightMotor = Motor.A;
    static EV3ColorSensor leftColor = new EV3ColorSensor(SensorPort.S2);
    static EV3ColorSensor rightColor  = new EV3ColorSensor(SensorPort.S1);
    static int BLACK = 7;

    public static void main(String[] args) {
        motorInit();
        while ( ! Button.ESCAPE.isDown() ) {
            if ( rightColor.getColorID() == BLACK && leftColor.getColorID() == BLACK ) {
                motorSet(0, 0);
            }
            if ( rightColor.getColorID() == BLACK && leftColor.getColorID() != BLACK ) {
                motorSet(150, 30);
            }
            if ( rightColor.getColorID() != BLACK && leftColor.getColorID() == BLACK ) {
                motorSet(30, 150);
            }
            if ( rightColor.getColorID() != BLACK && leftColor.getColorID() != BLACK ) {
                motorSet(150, 150);
            }
        }
    }

    private static void motorInit() {
        leftMotor.resetTachoCount();
        rightMotor.resetTachoCount();
        leftMotor.rotateTo(0);
        rightMotor.rotateTo(0);
    }

    private static void motorSet(int lMotorPow, int rMotorPow) {
        leftMotor.setSpeed(lMotorPow);
        rightMotor.setSpeed(rMotorPow);

        if ( lMotorPow > 0 ) {
            leftMotor.forward();
        }
        else if ( lMotorPow < 0 ) {
            leftMotor.backward();
        }
        else {
            leftMotor.stop();
        }

        if ( rMotorPow > 0 ) {
            rightMotor.forward();
        }
        else if ( rMotorPow < 0 ) {
            rightMotor.backward();
        }
        else {
            rightMotor.stop();
        }
    }
}
