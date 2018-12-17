package steelfight;

import java.util.ArrayList;
import java.util.List;

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
        List<CourseParameter[]> selectCourse = new ArrayList<>();
        CourseParameter[] courseParams;
        CourseParameter[] courseParamsQ = new CourseParameter[26];
        //                                   ( 距離, 時間,  ｿﾅｰ,   色, ﾄﾚﾓ, 前進, 旋回,  ｱｰﾑ,  PID );
        courseParamsQ[0]  = new CourseParameter(    0,  100,    0,    0,   0,   10,  100,    1,   38 );
        courseParamsQ[1]  = new CourseParameter(  600,    0,    0,    0,   2,   70,    0,    1,   38 );
        courseParamsQ[2]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   38 );
        courseParamsQ[3]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -16 );
        courseParamsQ[4]  = new CourseParameter( 1200,    0,    0,    0,   2,   70,    0,    1,  -16 );
        courseParamsQ[5]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -16 );
        courseParamsQ[6]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsQ[7]  = new CourseParameter( 1500,    0,    0,    0,   2,  -70,    0,    1,   -1 );
        courseParamsQ[8]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsQ[9]  = new CourseParameter(    0,  300,    0,    0,   0,    0,    0,    1,   -1 );
        courseParamsQ[10] = new CourseParameter(  270,    0,    0,    0,   2,   20,    0,    1,  -28 );
        courseParamsQ[11] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,    1 );
        courseParamsQ[12] = new CourseParameter( 1200,    0,    0,    0,   2,   70,    0,    1,    1 );
        courseParamsQ[13] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,    1 );
        courseParamsQ[14] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsQ[15] = new CourseParameter( 1500,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsQ[16] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsQ[17] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -42 );
        courseParamsQ[18] = new CourseParameter(  450,    0,    0,    0,   2,   70,    0,    1,  -42 );
        courseParamsQ[19] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -42 );
        courseParamsQ[20] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   18 );
        courseParamsQ[21] = new CourseParameter( 1120,    0,    0,    0,   2,   70,    0,    1,   18 );
        courseParamsQ[22] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   18 );
        courseParamsQ[23] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -3 );
        courseParamsQ[24] = new CourseParameter(    0,10000,    0,    5,   2,  -70,    0,    1,   -3 );
        courseParamsQ[25] = new CourseParameter(    0,    0,    0,    0,  -1,    0,    0,    0,    0 );

        CourseParameter[] courseParamsFinal = new CourseParameter[26];
        //                                         ( 距離, 時間,  ｿﾅｰ,   色, ﾄﾚﾓ, 前進, 旋回,  ｱｰﾑ,  PID );
        courseParamsFinal[0]  = new CourseParameter(    0,  100,    0,    0,   0,   10,  100,    1,   38 );
        courseParamsFinal[1]  = new CourseParameter(  600,    0,    0,    0,   2,   70,    0,    1,   38 );
        courseParamsFinal[2]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   38 );
        courseParamsFinal[3]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -16 );
        courseParamsFinal[4]  = new CourseParameter( 1200,    0,    0,    0,   2,   70,    0,    1,  -16 );
        courseParamsFinal[5]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -16 );
        courseParamsFinal[6]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal[7]  = new CourseParameter( 1500,    0,    0,    0,   2,  -70,    0,    1,   -1 );
        courseParamsFinal[8]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal[9]  = new CourseParameter(    0,  300,    0,    0,   0,    0,    0,    1,   -1 );
        courseParamsFinal[10] = new CourseParameter(  270,    0,    0,    0,   2,   20,    0,    1,  -28 );
        courseParamsFinal[11] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,    1 );
        courseParamsFinal[12] = new CourseParameter( 1200,    0,    0,    0,   2,   70,    0,    1,    1 );
        courseParamsFinal[13] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,    1 );
        courseParamsFinal[14] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal[15] = new CourseParameter( 1500,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsFinal[16] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal[17] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -42 );
        courseParamsFinal[18] = new CourseParameter(  450,    0,    0,    0,   2,   70,    0,    1,  -42 );
        courseParamsFinal[19] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -42 );
        courseParamsFinal[20] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   18 );
        courseParamsFinal[21] = new CourseParameter( 1120,    0,    0,    0,   2,   70,    0,    1,   18 );
        courseParamsFinal[22] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   18 );
        courseParamsFinal[23] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -3 );
        courseParamsFinal[24] = new CourseParameter(    0,10000,    0,    5,   2,  -70,    0,    1,   -3 );
        courseParamsFinal[25] = new CourseParameter(    0,    0,    0,    0,  -1,    0,    0,    0,    0 );

        CourseParameter[] courseParamsArmTest = new CourseParameter[10];
        //                                        ( 距離, 時間,  ｿﾅｰ,   色, ﾄﾚﾓ, 前進, 旋回,  ｱｰﾑ,  PID );
        courseParamsArmTest[0]  = new CourseParameter( 1000,    0,    7,    0,   4,   25,    0,    1,    0 );
        courseParamsArmTest[1]  = new CourseParameter(   70,    0,    0,    0,   0,   10,    0,    1,    0 );
        courseParamsArmTest[2]  = new CourseParameter(    0, 2500,    0,    0,   0,    0,    0,    0,    0 );
        courseParamsArmTest[3]  = new CourseParameter(   50,    0,    0,    0,   2,  -10,    0,    0,    0 );
        courseParamsArmTest[4]  = new CourseParameter( 1000,    0,    0,    0,   2,  -70,    0,    0,    0 );
        courseParamsArmTest[5]  = new CourseParameter(  200,    0,    0,    0,   2,   10,    0,    0,    0 );
        courseParamsArmTest[6]  = new CourseParameter(    0,    0,    0,    2,   2,   50,    0,    0,    0 );
        courseParamsArmTest[7]  = new CourseParameter(  200,    0,    0,    0,   2,   10,    0,    0,    0 );
        courseParamsArmTest[8]  = new CourseParameter(    0,    0,    0,    5,   2,  -50,    0,    0,  180 );
        courseParamsArmTest[9]  = new CourseParameter(    0,    0,    0,    0,  -1,    0,    0,    0,    0 );

        CourseParameter[] courseParamsLineTest = new CourseParameter[2];
        //                                            ( 距離, 時間,  ｿﾅｰ,   色, ﾄﾚﾓ, 前進, 旋回,  ｱｰﾑ,  PID );
        courseParamsLineTest[0]  = new CourseParameter( 2000,    0,    0,    0,   1,   50,    0,    0,   57 );
        courseParamsLineTest[1]  = new CourseParameter(    0,    0,    0,    0,  -1,    0,    0,    0,    0 );

//      selectCourse.add(courseParamsLineTest);
        selectCourse.add(courseParamsQ);
        selectCourse.add(courseParamsFinal);
        selectCourse.add(courseParamsArmTest);
        selectCourse.add(courseParamsLineTest);

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
        PID pidLine      = new PID(0.3700F, 0.0200F,0.1000F); /* ライントレース用PID */
        PID pidLineBack = new PID(0.5000F, 0.0000F,0.0000F); /* ライントレース用PID */
        PID pidGyro      = new PID(1.0000F, 0.0005F, 0.0700F); /* ジャイロトレース用PID */
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
        int selectCourseNumbar = 0;
        int reverseSwitch = 0;
        int sonerTmp = 0;
        int runningTime = 600000;

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
            LCD.drawString("Ready?  " + selectCourseNumbar, 0, 0);
            LCD.drawString("Red: " + red, 0, 1);
            LCD.drawString("Gre: " + green, 0, 2);
            LCD.drawString("Blu: " + blue, 0, 3);
            LCD.drawString("RGB: " + colorSum, 0, 4);
            LCD.drawString("Col: " + colorId, 0, 5);
            LCD.drawString("Gyr: " + gyroInt, 0, 6);
            LCD.drawString("REVERS: " + reverseSwitch, 0, 7);

            if ( Button.UP.isDown() ) {
                if (selectCourseNumbar >= selectCourse.size() - 1) {
                    selectCourseNumbar = 0;
                }
                else {
                    selectCourseNumbar++;
                }
            }

            if ( Button.DOWN.isDown() ) {
                if ( reverseSwitch == 0 ) {
                    reverseSwitch = 1;
                }
                else if ( reverseSwitch == 1) {
                    reverseSwitch = 0;
                }
            }

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

        courseParams = selectCourse.get(selectCourseNumbar);
        if (selectCourseNumbar != 0) {
            runningTime = 120000;
        }

        // アームモーターリセット
        armMotorInit();

        // タイマー計測開始
        counter.reset();
        stopwatch.reset();
        if (reverseSwitch == 1) {
            sonerSearchS = 1;

        }
        sonerTmp = 75;

        if (courseParams[courseNumber].getTraceMode() == 4) {
            sonerSearchS = 1;
        }

        // 走行（10ms周期で実行）60秒経過すると終了する
        while ( ! Button.ESCAPE.isDown() && counter.elapsed() < runningTime && courseParams[courseNumber].getTraceMode() != -1) {
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
                sonerTmp = 75;
                if (courseParams[courseNumber].getTraceMode() == 4) {
                    sonerSearchS = 1;
                }
                else {
                    sonerSearchS = 0;
                }
            }

            // 区間情報から値を取得する
            forward = courseParams[courseNumber].getForward();
            turn = courseParams[courseNumber].getTurn();
            if (reverseSwitch == 1 && courseParams[courseNumber].getTraceMode() == 0) {
                turn = courseParams[courseNumber].getTurn() * (-1);
            }


            if (courseParams[courseNumber].getTraceMode() == 1) {
                int lineTurn;
                if ( forward >= 0 ) {
                    lineTurn =  pidLine.calcControl(courseParams[courseNumber].getPidTarget() - colorSum);
                }
                else {
                    lineTurn = pidLineBack.calcControl(colorSum - courseParams[courseNumber].getPidTarget());
                }
                if (reverseSwitch == 1) {
                    lineTurn = lineTurn * (-1);
                }
                turn = turn + lineTurn;
            }
            else if (courseParams[courseNumber].getTraceMode() == 2) {
                int targetGyro = courseParams[courseNumber].getPidTarget();
                if (reverseSwitch == 1) {
                    targetGyro = targetGyro * (-1);
                }
                if (reverseSwitch == 0) {
                    turn = turn + pidGyro.calcControl(targetGyro - gyroInt);
                }
                else if (reverseSwitch == 1) {
                    turn = turn + (pidGyro.calcControl(gyroInt - targetGyro) * (-1));
                }
                if ( forward < 0 ) {
                    turn = turn * (-1);
                }
            }
            else if (courseParams[courseNumber].getTraceMode() == 3 || courseParams[courseNumber].getTraceMode() == 4) {
                if (search == 0) {
                    if (100 > gyroInt - tmpGyro && sonerSearchS == 0) {
                        turn = 100;
                        forward = 5;
                    }
                    else if (sonerSearchS == 0){
                        sonerSearchS = 1;
                    }

                    if (-100 < gyroInt - tmpGyro && sonerSearchS == 1) {
                        turn = -100;
                        forward = 5;
                    }
                    else if (sonerSearchS == 1){
                        sonerSearchS = 0;
                    }

                    if (sonicInt < sonerTmp) {
                        search = 1;
                        sonicInt = sonerTmp;
                    }
                }
                else if (search == 1) {
                    if (sonicInt <= sonerTmp) {
                        sonicInt = sonerTmp;
                    }
                    else {
                        search = 0;
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

            // モーター制御
            armSet(courseParams[courseNumber].getArmMode());
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
            if (armMotor.getTachoCount() > 0) {
                armMotorSet(-600);
            }
            else {
                armMotorSet(0);
            }
        }
        if (armMode == 1) {
            if (armMotor.getTachoCount() < 650) {
                armMotorSet(600);
            }
            else {
                armMotorSet(0);
            }
        }
    }
}
