package steelfight;

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
    static final RegulatedMotor armMotor = Motor.A;
    static final RegulatedMotor leftMotor = Motor.B;
    static final RegulatedMotor rightMotor = Motor.C;
    static final EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
    static final EV3UltrasonicSensor sonicSensor = new EV3UltrasonicSensor(SensorPort.S2);
    static final EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);
    static final EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S4);

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
        SecondCounter counter = new SecondCounter();
        PID pidLine = new PID(1.2500F, 0.0001F, 0.1700F);
        PID pidGyro = new PID(1.0F, 0.0005F, 0.07F);
        int forward = 0;
        int turn = 0;
        int red = 0;
        int green = 0;
        int blue = 0;
        int colorSum = 0;
        int sonicInt = 0;
        int gyroInt = 0;
        int touchInt = 0;
        int armflag = 0;
        motorInit();
        gyroSensor.reset();
        while ( ! Button.ENTER.isDown() ) {
            // センサーの値取得
            color.fetchSample(colorValue, 0);
            sonic.fetchSample(sonicValue, 0);
            gyro.fetchSample(gyroValue, 0);
            touch.fetchSample(touchValue, 0);
            // int型に変換
            red = (int)(colorValue[0] * 100);
            green = (int)(colorValue[1] * 100);
            blue = (int)(colorValue[2] * 100);
            colorSum = (int)(red + green + blue);
            sonicInt = (int)(sonicValue[0] * 100);
            gyroInt = (int)(gyroValue[0]);
            touchInt = (int)(touchValue[0]);
            LCD.clear();
            LCD.drawString("Ready?", 0, 0);
            LCD.drawString("Red: " + red, 0, 1);
            LCD.drawString("Gre: " + green, 0, 2);
            LCD.drawString("Blu: " + blue, 0, 3);
            LCD.drawString("RGB: " + colorSum, 0, 4);
            LCD.drawString("Son: " + sonicInt, 0, 5);
            LCD.drawString("Gyr: " + gyroInt, 0, 6);
            LCD.drawString("Arm: " + armMotor.getTachoCount(), 0, 7);
//            LCD.drawString("Tou: " + touchInt, 0, 7);
            if ( Button.LEFT.isDown() ) {
                armMotorSet(200);
            }
            else if ( Button.RIGHT.isDown() ) {
                armMotorSet(-200);
            }
            else {
                armMotorSet(0);
            }
            Delay.msDelay(100);
        }
        armMotorInit();
        counter.start();
        while ( ! Button.ESCAPE.isDown()  && counter.getSecond() < 60) {
            color.fetchSample(colorValue, 0);
            red = (int)(colorValue[0] * 100);
            green = (int)(colorValue[1] * 100);
            blue = (int)(colorValue[2] * 100);
            colorSum = (int)(red + green + blue);
            sonic.fetchSample(sonicValue, 0);
            gyro.fetchSample(gyroValue, 0);
            touch.fetchSample(touchValue, 0);
            // int型に変換
            red = (int)(colorValue[0] * 100);
            green = (int)(colorValue[1] * 100);
            blue = (int)(colorValue[2] * 100);
            colorSum = (int)(red + green + blue);
            sonicInt = (int)(sonicValue[0] * 100);
            gyroInt = (int)(gyroValue[0]);
            touchInt = (int)(touchValue[0]);

            forward = 50;

            // ライントレース
            turn = pidLine.calcControl(70 - colorSum);

            // ジャイロトレース
//                if (counter.getSecond() < 10) {
//                    turn = pidGyro.calcControl(0 - gyroInt);
//                }
//                else {
//                    turn = pidGyro.calcControl(90 - gyroInt);
//                }

            LCD.clear();
            LCD.drawString("Go !!", 0, 0);
            LCD.drawString("Red: " + red, 0, 1);
            LCD.drawString("Gre: " + green, 0, 2);
            LCD.drawString("Blu: " + blue, 0, 3);
            LCD.drawString("RGB: " + colorSum, 0, 4);
            LCD.drawString("Son: " + sonicInt, 0, 5);
            LCD.drawString("Gyr: " + gyroInt, 0, 6);
            LCD.drawString("Tur: " + turn, 0, 7);
//            LCD.drawString("Tou: " + touchInt, 0, 7);

            if ( Button.LEFT.isDown() ) {
                armflag = 0;
            }
            else if ( Button.RIGHT.isDown() ) {
                armflag = 1;
            }

            armSet(armflag);
            steeringRun(forward, turn);

            Delay.msDelay(10);
        }
        counter.stop();
    }

    private static void motorInit() {
        armMotorInit();
        leftMotor.resetTachoCount();
        leftMotor.rotateTo(0);
        rightMotor.resetTachoCount();
        rightMotor.rotateTo(0);
    }

    private static void armMotorInit() {
        armMotor.resetTachoCount();
        armMotor.rotateTo(0);
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

    public static void steeringRun(int forward, int turn) {
        int rightMotorPower = forward;
        int leftMotorPower = forward;
        if (turn > 0) {
            rightMotorPower = forward - (forward * (2 * turn) / 100);
        }
        if (turn < 0) {
            turn = turn * (-1);
            leftMotorPower = forward - (forward * (2 * turn) / 100);
        }
        motorSet(rightMotorPower * 10, leftMotorPower * 10);
    }

    public static void armMotorSet(int armMotorPow) {
        armMotor.setSpeed(armMotorPow);

        if ( armMotorPow > 0 ) {
            armMotor.forward();
        }
        else if ( armMotorPow < 0 ) {
            armMotor.backward();
        }
        else {
            armMotor.stop();
        }
    }

    public static void armSet(int armflag) {
        if (armflag == 0) {
            if (armMotor.getTachoCount() < 0) {
                armMotorSet(600);
            }
            else {
                armMotorSet(0);
            }
        }
        if (armflag == 1) {
            if (armMotor.getTachoCount() > -650) {
                armMotorSet(-600);
            }
            else {
                armMotorSet(0);
            }
        }
    }
}
