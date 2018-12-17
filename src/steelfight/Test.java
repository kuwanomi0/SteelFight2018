package steelfight;

import lejos.hardware.Button;
import lejos.hardware.Sound;
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
import lejos.utility.Stopwatch;

public class Test {
    static final RegulatedMotor armMotor = Motor.A;
    static final RegulatedMotor leftMotor = Motor.B;
    static final RegulatedMotor rightMotor = Motor.C;
    static final EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
    static final EV3UltrasonicSensor sonicSensor = new EV3UltrasonicSensor(SensorPort.S2);
    static final EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);
    static final EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S4);

    public static void main(String[] args) {
        // 区間情報
        int courseNumber = 0;
        CourseParameter[] courseParams = new CourseParameter[24];
        //                                   ( 距離, 時間,  ｿﾅｰ,   色, ﾄﾚﾓ, 前進, 旋回,  ｱｰﾑ,  PID );
        courseParams[0]  = new CourseParameter(    0,  100,    0,    0,   2,   10,  100,    0,   38 );
        courseParams[1]  = new CourseParameter(  600,    0,    0,    0,   2,   70,    0,    0,   38 );
        courseParams[2]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    0,   38 );
        courseParams[3]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    0,  -16 );
        courseParams[4]  = new CourseParameter( 1200,    0,    0,    0,   2,   70,    0,    0,  -16 );
        courseParams[5]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    0,  -16 );
        courseParams[6]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    0,   -1 );
        courseParams[7]  = new CourseParameter( 1500,    0,    0,    0,   2,  -70,    0,    0,   -1 );
        courseParams[8]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    0,   -1 );
        courseParams[9]  = new CourseParameter(    0,  300,    0,    0,   0,    0,    0,    0,   -1 );
        courseParams[10] = new CourseParameter(  270,    0,    0,    0,   2,   20,    0,    0,  -28 );
        courseParams[11] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    0,    1 );
        courseParams[12] = new CourseParameter( 1200,    0,    0,    0,   2,   70,    0,    0,    1 );
        courseParams[13] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    0,    1 );
        courseParams[14] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    0,    0 );
        courseParams[15] = new CourseParameter( 1500,    0,    0,    0,   2,  -70,    0,    0,    0 );
        courseParams[16] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    0,    0 );
        courseParams[17] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    0,  -41 );
        courseParams[18] = new CourseParameter(  450,    0,    0,    0,   2,   70,    0,    0,  -41 );
        courseParams[19] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    0,  -41 );
        courseParams[20] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    0,   18 );
        courseParams[21] = new CourseParameter( 1120,    0,    0,    0,   2,   70,    0,    0,   18 );
        courseParams[22] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    0,   18 );
        courseParams[22] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    0,   -1 );
        courseParams[22] = new CourseParameter(    0,10000,    0,    5,   2,  -70,    0,    0,   -1 );
        courseParams[23] = new CourseParameter(    0,    0,    0,    0,  -1,    0,    0,    0,    0 );


        // 使用するセンサー定義
        SensorMode color = colorSensor.getMode(2);
        SensorMode sonic = sonicSensor.getMode(0);
        SensorMode gyro = gyroSensor.getMode(1);
        SensorMode touch = touchSensor.getMode(0);
        // センサーの取得地を格納する配列
        float[] colorValue = new float[color.sampleSize()];
        float[] sonicValue = new float[sonic.sampleSize()];
        float[] gyroValue = new float[gyro.sampleSize()];
        float[] touchValue = new float[touch.sampleSize()];
        Stopwatch counter = new Stopwatch();
        Stopwatch stopwatch = new Stopwatch();
        PID pidLine = new PID(1.2500F, 0.0001F, 0.1700F); /* ライントレース用PID */
        PID pidGyro = new PID(1.0000F, 0.0005F, 0.0700F); /* ジャイロトレース用PID */
        Distance dis = new Distance();
        ColorPanel col = new ColorPanel();
        int forward = 0;
        int turn = 0;
        int red = 0;
        int green = 0;
        int blue = 0;
        int colorSum = 0;
        int colorId = 0;
        int sonicInt = 0;
        int gyroInt = 0;
        int touchInt = 0;
        int armMode = 0;
        int beforeDistance = 0;
        int tmpGyro = 0;
        int search = 0;
        int sonerSearchS = 0;

        // モジュール初期化
        motorInit();
        gyroSensor.reset();

        Sound.beep();

        // スタート待機 (20ms周期で実行)
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

            tmpGyro = gyroInt;


            // 色の判定
            colorId = col.decision(red, green, blue);


            // LCD出力
            LCD.clear();
            LCD.drawString("Ready?", 0, 0);
            LCD.drawString("Red: " + red, 0, 1);
            LCD.drawString("Gre: " + green, 0, 2);
            LCD.drawString("Blu: " + blue, 0, 3);
            LCD.drawString("RGB: " + colorSum, 0, 4);
            LCD.drawString("Col: " + colorId, 0, 5);
            LCD.drawString("Gyr: " + gyroInt, 0, 6);
            LCD.drawString("Arm: " + armMotor.getTachoCount(), 0, 7);

            // アーム調整
            if ( Button.LEFT.isDown() ) {
                armMotorSet(200);
            }
            else if ( Button.RIGHT.isDown() ) {
                armMotorSet(-200);
            }
            else {
                armMotorSet(0);
            }

            // 20ms周期で実行
            Delay.msDelay(20);
        }

        // アームモーターリセット
        armMotorInit();

        // タイマー計測開始
        counter.reset();
        stopwatch.reset();

        // 走行（10ms周期で実行）60秒経過すると終了する
        while ( ! Button.ESCAPE.isDown() && counter.elapsed() < 600000 && courseParams[courseNumber].getTraceMode() != -1) {
            // センサー取得
            color.fetchSample(colorValue, 0);
            sonic.fetchSample(sonicValue, 0);
            gyro.fetchSample(gyroValue, 0);
            touch.fetchSample(touchValue, 0);

            // 走行距離計算
            dis.update(leftMotor.getTachoCount(), rightMotor.getTachoCount());

            // int型に変換
            red = (int)(colorValue[0] * 100);
            green = (int)(colorValue[1] * 100);
            blue = (int)(colorValue[2] * 100);
            colorSum = (int)(red + green + blue);
            sonicInt = (int)(sonicValue[0] * 100);
            gyroInt = (int)(gyroValue[0]);
            touchInt = (int)(touchValue[0]);

            // 色の判定
            colorId = col.decision(red, green, blue);

            // 区間情報変更の確認
            if (courseChange(courseParams[courseNumber], dis.getDistance() - beforeDistance, stopwatch.elapsed(), sonicInt, colorId)) {
                courseNumber++;
                beforeDistance = dis.getDistance();
                stopwatch.reset();
                //ビープ音を鳴らす
                Sound.beep();
                tmpGyro = gyroInt;
            }

            // 区間情報から値を取得する
            forward = courseParams[courseNumber].getForward();
            turn = courseParams[courseNumber].getTurn();


            if (courseParams[courseNumber].getTraceMode() == 1) {
                turn = turn + pidLine.calcControl(courseParams[courseNumber].getPidTarget() - colorSum);
                if ( forward < 0 ) {
                    turn = turn * (-1);
                }
            }
            else if (courseParams[courseNumber].getTraceMode() == 2) {
                turn = turn + pidGyro.calcControl(courseParams[courseNumber].getPidTarget() - gyroInt);
                if ( forward < 0 ) {
                    turn = turn * (-1);
                }
            }
            else if (courseParams[courseNumber].getTraceMode() == 3) {
                if (search == 0) {
                    if (45 > gyroInt - tmpGyro && sonerSearchS == 0) {
                        turn = 100;
                        forward = 10;
                    }
                    else if (sonerSearchS == 0){
                        sonerSearchS = 1;
                    }

                    if (-45 < gyroInt - tmpGyro && sonerSearchS == 1) {
                        turn = -100;
                        forward = 10;
                    }
                    else if (sonerSearchS == 1){
                        sonerSearchS = 0;
                    }

                    if (sonicInt > 100) {

                    }
                }
            }

            // LCD表示
            LCD.clear();
            LCD.drawString("Go !!", 0, 0);
            LCD.drawString("Red: " + red, 0, 1);
            LCD.drawString("Gre: " + green, 0, 2);
            LCD.drawString("Blu: " + blue, 0, 3);
            LCD.drawString("RGB: " + courseNumber, 0, 4);
            LCD.drawString("Son: " + sonicInt, 0, 5);
            LCD.drawString("Gyr: " + gyroInt, 0, 6);
            LCD.drawString("Dis: " + dis.getDistance(), 0, 7);

            // アーム手動切り替え
            if ( Button.LEFT.isDown() ) {
                armMode = 0;
            }
            else if ( Button.RIGHT.isDown() ) {
                armMode = 1;
            }

            // モーター制御
            armSet(armMode);
            steeringRun(forward, turn);

            // 10ms周期で実行
            Delay.msDelay(10);
        }
    }

    /**
     * 区間変更条件確認
     * @param courseParameter
     * @param dis
     * @param second
     * @param sonic
     * @param colorId
     * @return
     */
    private static boolean courseChange(CourseParameter courseParameter, float dis, int second, int sonic, int colorId) {
        boolean changed = false;
        if (courseParameter.getDis() != 0) {
            if (dis < 0) {
                dis = dis * (-1);
            }
            if (dis >= courseParameter.getDis()) {
                changed = true;
            }
        }

        if (courseParameter.getTime() != 0) {
            if (second >= courseParameter.getTime()) {
                changed = true;
            }
        }

        if (courseParameter.getSonarDis() != 0) {
            if (0 < sonic && sonic <= courseParameter.getSonarDis()) {
                changed = true;
            }
        }

        if (courseParameter.getColorId() != 0) {
            if (courseParameter.getColorId() == colorId) {
                changed = true;
            }
        }

        return changed;
    }

    /**
     * モーター初期化
     */
    private static void motorInit() {
        armMotorInit();
        runMotorInit();
    }

    /**
     * 走行モーター初期化
     */
    private static void runMotorInit() {
        leftMotor.resetTachoCount();
        leftMotor.rotateTo(0);
        rightMotor.resetTachoCount();
        rightMotor.rotateTo(0);
    }

    /**
     * アームモーター初期化
     */
    private static void armMotorInit() {
        armMotor.resetTachoCount();
        armMotor.rotateTo(0);
    }

    /**
     * 走行モーター制御
     * @param lMotorPow
     * @param rMotorPow
     */
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

    /**
     * ステアリング走行
     * @param forward
     * @param turn
     */
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

    /**
     * アームモーター制御
     * @param armMotorPow
     */
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

    /**
     * アームモーター設定
     * @param armMode
     */
    public static void armSet(int armMode) {
        if (armMode == 0) {
            if (armMotor.getTachoCount() < 0) {
                armMotorSet(600);
            }
            else {
                armMotorSet(0);
            }
        }
        if (armMode == 1) {
            if (armMotor.getTachoCount() > -650) {
                armMotorSet(-600);
            }
            else {
                armMotorSet(0);
            }
        }
    }
}
