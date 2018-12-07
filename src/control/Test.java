package control;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class Test {
    static RegulatedMotor armMotor = Motor.A;
    static RegulatedMotor leftMotor = Motor.B;
    static RegulatedMotor rightMotor = Motor.C;
    static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
    static EV3UltrasonicSensor sonicSensor = new EV3UltrasonicSensor(SensorPort.S2);
    static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);
    static EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S4);

    public static void main(String[] args) {
        // 使用するセンサー定義
        SensorMode color = colorSensor.getMode(2);
        SensorMode sonic = sonicSensor.getMode(0);
        SensorMode gyro = gyroSensor.getMode(1);
        SensorMode touch = touchSensor.getMode(0);
        // センサーの取得地を格納する配列
        float colorValue[] = new float[color.sampleSize()];
        float sonicValue[] = new float[sonic.sampleSize()];
        float gyroValue[] = new float[gyro.sampleSize()];
        float touchValue[] = new float[touch.sampleSize()];
        motorInit();
        gyroSensor.reset();
        LCD.clear();
        LCD.drawString("Ready?", 0, 0);
        while ( ! Button.ENTER.isDown() ) {
            // センサーの値取得
            color.fetchSample(colorValue, 0);
            sonic.fetchSample(sonicValue, 0);
            gyro.fetchSample(gyroValue, 0);
            touch.fetchSample(touchValue, 0);
            LCD.drawString("R: " + colorValue[0], 0, 1);
            LCD.drawString("G: " + colorValue[1], 0, 2);
            LCD.drawString("B: " + colorValue[2], 0, 3);
            LCD.drawString("S: " + sonicValue[0], 0, 4);
            LCD.drawString("G: " + gyroValue[0], 0, 5);
            LCD.drawString("T: " + touchValue[0], 0, 6);
            Delay.msDelay(100);
        }
        LCD.drawString("Go !!", 0, 0);
        while ( ! Button.ESCAPE.isDown() ) {
            motorSet(500, 500);
            color.fetchSample(colorValue, 0);
            sonic.fetchSample(sonicValue, 0);
            gyro.fetchSample(gyroValue, 0);
            touch.fetchSample(touchValue, 0);
            LCD.drawString("R: " + colorValue[0], 0, 1);
            LCD.drawString("G: " + colorValue[1], 0, 2);
            LCD.drawString("B: " + colorValue[2], 0, 3);
            LCD.drawString("S: " + sonicValue[0], 0, 4);
            LCD.drawString("G: " + gyroValue[0], 0, 5);
            LCD.drawString("T: " + touchValue[0], 0, 6);
            Delay.msDelay(100);
        }
    }

    private static void motorInit() {
        armMotor.resetTachoCount();
        armMotor.rotateTo(0);
        leftMotor.resetTachoCount();
        leftMotor.rotateTo(0);
        rightMotor.resetTachoCount();
        rightMotor.rotateTo(0);
    }

    public static void motorSet(int lMotorPow, int rMotorPow) {
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
