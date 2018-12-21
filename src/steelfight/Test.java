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
    static PID armPID = new PID(3.5f, 0.0f, 1.0f);
    static int armAngle = 0;

    public static void main(String[] args) {
        // 区間情報
        int courseNumber = 0;
        List<CourseParameter[]> selectCourse = new ArrayList<>();
        CourseParameter[] courseParams;
        CourseParameter[] courseParamsQ = new CourseParameter[26];
        //                                     ( 距離, 時間,  ｿﾅｰ,   色, ﾄﾚﾓ, 前進, 旋回,  ｱｰﾑ,  PID );
        courseParamsQ[0]  = new CourseParameter(    0,   70,    0,    0,   0,   10,  100,    1,   38 );
        courseParamsQ[1]  = new CourseParameter(  580,    0,    0,    0,   2,   50,    0,    1,   38 );
        courseParamsQ[2]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   38 );
        courseParamsQ[3]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsQ[4]  = new CourseParameter( 1180,    0,    0,    0,   2,   70,    0,    1,  -17 );
        courseParamsQ[5]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsQ[6]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsQ[7]  = new CourseParameter( 1480,    0,    0,    0,   2,  -70,    0,    1,   -1 );
        courseParamsQ[8]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsQ[9]  = new CourseParameter(    0,  300,    0,    0,   0,    0,    0,    1,   -1 );
        courseParamsQ[10] = new CourseParameter(  240,    0,    0,    0,   2,   30,    0,    1,  -25 );
        courseParamsQ[11] = new CourseParameter(  100,    0,    0,    0,   2,   30,    0,    1,    1 );
        courseParamsQ[12] = new CourseParameter( 1300,    0,    0,    0,   2,   70,    0,    1,    1 );
        courseParamsQ[13] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,    1 );
        courseParamsQ[14] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsQ[15] = new CourseParameter( 1550,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsQ[16] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsQ[17] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsQ[18] = new CourseParameter(  450,    0,    0,    0,   2,   70,    0,    1,  -44 );
        courseParamsQ[19] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsQ[20] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        courseParamsQ[21] = new CourseParameter( 1200,    0,    0,    0,   2,   70,    0,    1,   19 );
        courseParamsQ[22] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        courseParamsQ[23] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -3 );
        courseParamsQ[24] = new CourseParameter(    0,10000,    0,    5,   2,  -70,    0,    1,   -3 );
        courseParamsQ[25] = new CourseParameter(    0,    0,    0,    0,  -1,    0,    0,    0,    0 );

        CourseParameter[] courseParamsFinal1 = new CourseParameter[49];
        //                                         ( 距離, 時間,  ｿﾅｰ,   色, ﾄﾚﾓ, 前進, 旋回,  ｱｰﾑ,  PID );
        courseParamsFinal1[0]  = new CourseParameter(    0,   70,    0,    0,   0,   10,  100,    1,   38 );
        courseParamsFinal1[1]  = new CourseParameter(  580,    0,    0,    0,   2,   50,    0,    1,   38 );
        courseParamsFinal1[2]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   38 );
        courseParamsFinal1[3]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsFinal1[4]  = new CourseParameter( 1180,    0,    0,    0,   2,   70,    0,    1,  -17 );
        courseParamsFinal1[5]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsFinal1[6]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal1[7]  = new CourseParameter( 1480,    0,    0,    0,   2,  -70,    0,    1,   -1 );
        courseParamsFinal1[8]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal1[9]  = new CourseParameter(    0,  300,    0,    0,   0,    0,    0,    1,   -1 );
        courseParamsFinal1[10] = new CourseParameter(  240,    0,    0,    0,   2,   30,    0,    1,  -25 );
        courseParamsFinal1[11] = new CourseParameter(  100,    0,    0,    0,   2,   30,    0,    1,    1 );
        courseParamsFinal1[12] = new CourseParameter( 1300,    0,    0,    0,   2,   70,    0,    1,    1 );
        courseParamsFinal1[13] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,    1 );
        courseParamsFinal1[14] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal1[15] = new CourseParameter( 1550,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsFinal1[16] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal1[17] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsFinal1[18] = new CourseParameter(  450,    0,    0,    0,   2,   70,    0,    1,  -44 );
        courseParamsFinal1[19] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsFinal1[20] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        courseParamsFinal1[21] = new CourseParameter( 1200,    0,    0,    0,   2,   70,    0,    1,   19 );
        courseParamsFinal1[22] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        // ここから残りの2本
        courseParamsFinal1[23] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,  -17 );
        courseParamsFinal1[24] = new CourseParameter( 1120,    0,    0,    0,   2,  -70,    0,    1,  -17 );
        // 一本目サーチ
        courseParamsFinal1[25] = new CourseParameter(    0,    0,   14,    0,   3,   50,    0,    1,    0 );
        courseParamsFinal1[26] = new CourseParameter(  150,    0,    0,    0,   0,   25,    0,    1,    0 );
        courseParamsFinal1[27] = new CourseParameter(    0, 2500,    0,    0,   0,    3,    0,    0,    0 );
        courseParamsFinal1[28] = new CourseParameter(  100,    0,    0,    3,   2,  -30,    0,    0,  110 );
        courseParamsFinal1[29] = new CourseParameter(    0,    0,    0,    3,   2,  -30,    0,    0,  110 );
        courseParamsFinal1[30] = new CourseParameter(    0,    0,    0,    3,   0,   10,    0,    0,    0 );
        courseParamsFinal1[31] = new CourseParameter(  100,    0,    0,    0,   2,   15,    0,    0,    0 );
        courseParamsFinal1[32] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal1[33] = new CourseParameter(    0,    0,    0,    2,   2,   70,    0,    1,    0 );
        courseParamsFinal1[34] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal1[35] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,  -17 );
        courseParamsFinal1[36] = new CourseParameter( 1020,    0,    0,    0,   2,  -70,    0,    1,  -17 );
        // 二本目サーチ
        courseParamsFinal1[37] = new CourseParameter(    0,    0,   14,    0,   3,   50,    0,    1,    0 );
        courseParamsFinal1[38] = new CourseParameter(  150,    0,    0,    0,   0,   25,    0,    1,    0 );
        courseParamsFinal1[39] = new CourseParameter(    0, 2500,    0,    0,   0,    3,    0,    0,    0 );
        courseParamsFinal1[40] = new CourseParameter(  100,    0,    0,    3,   2,  -30,    0,    0,  110 );
        courseParamsFinal1[41] = new CourseParameter(    0,    0,    0,    3,   2,  -30,    0,    0,  110 );
        courseParamsFinal1[42] = new CourseParameter(    0,    0,    0,    3,   0,   10,    0,    0,    0 );
        courseParamsFinal1[43] = new CourseParameter(  100,    0,    0,    0,   2,   15,    0,    0,    0 );
        courseParamsFinal1[44] = new CourseParameter(    0,  250,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal1[45] = new CourseParameter(    0,    0,    0,    2,   2,   70,    0,    1,    0 );
        courseParamsFinal1[46] = new CourseParameter(  400,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsFinal1[47] = new CourseParameter(    0, 2500,    0,    0,   0,    0,    0,    0,    0 );
        courseParamsFinal1[48] = new CourseParameter(    0,    0,    0,    0,  -1,    0,    0,    0,    0 );


        CourseParameter[] courseParamsFinal2 = new CourseParameter[49];
        //                                         ( 距離, 時間,  ｿﾅｰ,   色, ﾄﾚﾓ, 前進, 旋回,  ｱｰﾑ,  PID );
        courseParamsFinal2[0]  = new CourseParameter(    0,   70,    0,    0,   0,   10,  100,    1,   38 );
        courseParamsFinal2[1]  = new CourseParameter(  580,    0,    0,    0,   2,   50,    0,    1,   38 );
        courseParamsFinal2[2]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   38 );
        courseParamsFinal2[3]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsFinal2[4]  = new CourseParameter( 1180,    0,    0,    0,   2,   70,    0,    1,  -17 );
        courseParamsFinal2[5]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsFinal2[6]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal2[7]  = new CourseParameter( 1480,    0,    0,    0,   2,  -70,    0,    1,   -1 );
        courseParamsFinal2[8]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal2[9]  = new CourseParameter(    0,  300,    0,    0,   0,    0,    0,    1,   -1 );
        courseParamsFinal2[10] = new CourseParameter(  240,    0,    0,    0,   2,   30,    0,    1,  -25 );
        courseParamsFinal2[11] = new CourseParameter(  100,    0,    0,    0,   2,   30,    0,    1,    1 );
        courseParamsFinal2[12] = new CourseParameter( 1300,    0,    0,    0,   2,   70,    0,    1,    1 );
        courseParamsFinal2[13] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,    1 );
        courseParamsFinal2[14] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal2[15] = new CourseParameter( 1550,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsFinal2[16] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal2[17] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsFinal2[18] = new CourseParameter(  450,    0,    0,    0,   2,   70,    0,    1,  -44 );
        courseParamsFinal2[19] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsFinal2[20] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        courseParamsFinal2[21] = new CourseParameter( 1200,    0,    0,    0,   2,   70,    0,    1,   19 );
        courseParamsFinal2[22] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        // ここから残りの2本
        courseParamsFinal2[23] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   17 );
        courseParamsFinal2[24] = new CourseParameter( 1120,    0,    0,    0,   2,  -70,    0,    1,   17 );
        // 一本目サーチ
        courseParamsFinal2[25] = new CourseParameter(    0,    0,   14,    0,   4,   50,    0,    1,    0 );
        courseParamsFinal2[26] = new CourseParameter(  150,    0,    0,    0,   0,   25,    0,    1,    0 );
        courseParamsFinal2[27] = new CourseParameter(    0, 2500,    0,    0,   0,    3,    0,    0,    0 );
        courseParamsFinal2[28] = new CourseParameter(  100,    0,    0,    3,   2,  -30,    0,    0, -110 );
        courseParamsFinal2[29] = new CourseParameter(    0,    0,    0,    3,   2,  -30,    0,    0, -110 );
        courseParamsFinal2[30] = new CourseParameter(    0,    0,    0,    3,   0,   10,    0,    0,    0 );
        courseParamsFinal2[31] = new CourseParameter(  100,    0,    0,    0,   2,   15,    0,    0,    0 );
        courseParamsFinal2[32] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal2[33] = new CourseParameter(    0,    0,    0,    2,   2,   70,    0,    1,    0 );
        courseParamsFinal2[34] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal2[35] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   17 );
        courseParamsFinal2[36] = new CourseParameter( 1020,    0,    0,    0,   2,  -70,    0,    1,   17 );
        // 二本目サーチ
        courseParamsFinal2[37] = new CourseParameter(    0,    0,   14,    0,   4,   50,    0,    1,    0 );
        courseParamsFinal2[38] = new CourseParameter(  150,    0,    0,    0,   0,   25,    0,    1,    0 );
        courseParamsFinal2[39] = new CourseParameter(    0, 2500,    0,    0,   0,    3,    0,    0,    0 );
        courseParamsFinal2[40] = new CourseParameter(  100,    0,    0,    3,   2,  -30,    0,    0, -110 );
        courseParamsFinal2[41] = new CourseParameter(    0,    0,    0,    3,   2,  -30,    0,    0, -110 );
        courseParamsFinal2[42] = new CourseParameter(    0,    0,    0,    3,   0,   10,    0,    0,    0 );
        courseParamsFinal2[43] = new CourseParameter(  100,    0,    0,    0,   2,   15,    0,    0,    0 );
        courseParamsFinal2[44] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal2[45] = new CourseParameter(    0,    0,    0,    2,   2,   70,    0,    1,    0 );
        courseParamsFinal2[46] = new CourseParameter(  400,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsFinal2[47] = new CourseParameter(    0, 2500,    0,    0,   0,    0,    0,    0,    0 );
        courseParamsFinal2[48] = new CourseParameter(    0,    0,    0,    0,  -1,    0,    0,    0,    0 );


        CourseParameter[] courseParamsFinal3 = new CourseParameter[49];
        //                                         ( 距離, 時間,  ｿﾅｰ,   色, ﾄﾚﾓ, 前進, 旋回,  ｱｰﾑ,  PID );
        courseParamsFinal3[0]  = new CourseParameter(    0,   70,    0,    0,   0,   10,  100,    1,   38 );
        courseParamsFinal3[1]  = new CourseParameter(  580,    0,    0,    0,   2,   50,    0,    1,   38 );
        courseParamsFinal3[2]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   38 );
        courseParamsFinal3[3]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsFinal3[4]  = new CourseParameter( 1180,    0,    0,    0,   2,   70,    0,    1,  -17 );
        courseParamsFinal3[5]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsFinal3[6]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal3[7]  = new CourseParameter( 1480,    0,    0,    0,   2,  -70,    0,    1,   -1 );
        courseParamsFinal3[8]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal3[9]  = new CourseParameter(    0,  300,    0,    0,   0,    0,    0,    1,   -1 );
        courseParamsFinal3[10] = new CourseParameter(  240,    0,    0,    0,   2,   30,    0,    1,  -25 );
        courseParamsFinal3[11] = new CourseParameter(  100,    0,    0,    0,   2,   30,    0,    1,    1 );
        courseParamsFinal3[12] = new CourseParameter( 1300,    0,    0,    0,   2,   70,    0,    1,    1 );
        courseParamsFinal3[13] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,    1 );
        courseParamsFinal3[14] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal3[15] = new CourseParameter( 1550,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsFinal3[16] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal3[17] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsFinal3[18] = new CourseParameter(  450,    0,    0,    0,   2,   70,    0,    1,  -44 );
        courseParamsFinal3[19] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsFinal3[20] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        courseParamsFinal3[21] = new CourseParameter( 1200,    0,    0,    0,   2,   70,    0,    1,   19 );
        courseParamsFinal3[22] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        // ここから残りの2本
        courseParamsFinal3[23] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,  -17 );
        courseParamsFinal3[24] = new CourseParameter( 1120,    0,    0,    0,   2,  -70,    0,    1,  -17 );
        // 一本目サーチ
        courseParamsFinal3[25] = new CourseParameter(    0,    0,   14,    0,   3,   50,    0,    1,    0 );
        courseParamsFinal3[26] = new CourseParameter(  150,    0,    0,    0,   0,   25,    0,    1,    0 );
        courseParamsFinal3[27] = new CourseParameter(    0, 2500,    0,    0,   0,    3,    0,    0,    0 );
        courseParamsFinal3[28] = new CourseParameter(  100,    0,    0,    3,   2,  -30,    0,    0,  110 );
        courseParamsFinal3[29] = new CourseParameter(    0,    0,    0,    3,   2,  -30,    0,    0,  110 );
        courseParamsFinal3[30] = new CourseParameter(    0,    0,    0,    3,   0,   10,    0,    0,    0 );
        courseParamsFinal3[31] = new CourseParameter(  100,    0,    0,    0,   2,   15,    0,    0,    0 );
        courseParamsFinal3[32] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal3[33] = new CourseParameter(    0,    0,    0,    2,   2,   70,    0,    1,    0 );
        courseParamsFinal3[34] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal3[35] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   17 );
        courseParamsFinal3[36] = new CourseParameter( 1020,    0,    0,    0,   2,  -70,    0,    1,   17 );
        // 二本目サーチ
        courseParamsFinal3[37] = new CourseParameter(    0,    0,   14,    0,   4,   50,    0,    1,    0 );
        courseParamsFinal3[38] = new CourseParameter(  150,    0,    0,    0,   0,   25,    0,    1,    0 );
        courseParamsFinal3[39] = new CourseParameter(    0, 2500,    0,    0,   0,    3,    0,    0,    0 );
        courseParamsFinal3[40] = new CourseParameter(  100,    0,    0,    3,   2,  -30,    0,    0, -110 );
        courseParamsFinal3[41] = new CourseParameter(    0,    0,    0,    3,   2,  -30,    0,    0, -110 );
        courseParamsFinal3[42] = new CourseParameter(    0,    0,    0,    3,   0,   10,    0,    0,    0 );
        courseParamsFinal3[43] = new CourseParameter(  100,    0,    0,    0,   2,   15,    0,    0,    0 );
        courseParamsFinal3[44] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal3[45] = new CourseParameter(    0,    0,    0,    2,   2,   70,    0,    1,    0 );
        courseParamsFinal3[46] = new CourseParameter(  400,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsFinal3[47] = new CourseParameter(    0, 2500,    0,    0,   0,    0,    0,    0,    0 );
        courseParamsFinal3[48] = new CourseParameter(    0,    0,    0,    0,  -1,    0,    0,    0,    0 );



        CourseParameter[] courseParamsFinal4 = new CourseParameter[49];
        //                                          ( 距離, 時間,  ｿﾅｰ,   色, ﾄﾚﾓ, 前進, 旋回,  ｱｰﾑ,  PID );
        courseParamsFinal4[0]  = new CourseParameter(    0,   70,    0,    0,   0,   10,  100,    1,   38 );
        courseParamsFinal4[1]  = new CourseParameter(  580,    0,    0,    0,   2,   50,    0,    1,   38 );
        courseParamsFinal4[2]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   38 );
        courseParamsFinal4[3]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsFinal4[4]  = new CourseParameter( 1180,    0,    0,    0,   2,   70,    0,    1,  -17 );
        courseParamsFinal4[5]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsFinal4[6]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal4[7]  = new CourseParameter( 1480,    0,    0,    0,   2,  -70,    0,    1,   -1 );
        courseParamsFinal4[8]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal4[9]  = new CourseParameter(    0,  300,    0,    0,   0,    0,    0,    1,   -1 );
        courseParamsFinal4[10] = new CourseParameter(  240,    0,    0,    0,   2,   30,    0,    1,  -25 );
        courseParamsFinal4[11] = new CourseParameter(  100,    0,    0,    0,   2,   30,    0,    1,    1 );
        courseParamsFinal4[12] = new CourseParameter( 1300,    0,    0,    0,   2,   70,    0,    1,    1 );
        courseParamsFinal4[13] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,    1 );
        courseParamsFinal4[14] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal4[15] = new CourseParameter( 1550,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsFinal4[16] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal4[17] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsFinal4[18] = new CourseParameter(  450,    0,    0,    0,   2,   70,    0,    1,  -44 );
        courseParamsFinal4[19] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsFinal4[20] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        courseParamsFinal4[21] = new CourseParameter( 1200,    0,    0,    0,   2,   70,    0,    1,   19 );
        courseParamsFinal4[22] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        // ここから残りの2本
        courseParamsFinal4[23] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal4[24] = new CourseParameter( 1050,    0,    0,    0,   2,  -70,    0,    1,    0 );
        // 一本目サーチ
        courseParamsFinal4[25] = new CourseParameter(    0,    0,   14,    0,   3,   50,    0,    1,    0 );
        courseParamsFinal4[26] = new CourseParameter(  150,    0,    0,    0,   0,   25,    0,    1,    0 );
        courseParamsFinal4[27] = new CourseParameter(    0, 2500,    0,    0,   0,    3,    0,    0,    0 );
        courseParamsFinal4[28] = new CourseParameter(  100,    0,    0,    3,   2,  -30,    0,    0,  110 );
        courseParamsFinal4[29] = new CourseParameter(    0,    0,    0,    3,   2,  -30,    0,    0,  110 );
        courseParamsFinal4[30] = new CourseParameter(    0,    0,    0,    3,   0,   10,    0,    0,    0 );
        courseParamsFinal4[31] = new CourseParameter(  100,    0,    0,    0,   2,   15,    0,    0,    0 );
        courseParamsFinal4[32] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal4[33] = new CourseParameter(    0,    0,    0,    2,   2,   70,    0,    1,    0 );
        courseParamsFinal4[34] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal4[35] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,  -17 );
        courseParamsFinal4[36] = new CourseParameter( 1020,    0,    0,    0,   2,  -70,    0,    1,  -17 );
        // 二本目サーチ
        courseParamsFinal4[37] = new CourseParameter(    0,    0,   14,    0,   3,   50,    0,    1,    0 );
        courseParamsFinal4[38] = new CourseParameter(  150,    0,    0,    0,   0,   25,    0,    1,    0 );
        courseParamsFinal4[39] = new CourseParameter(    0, 2500,    0,    0,   0,    3,    0,    0,    0 );
        courseParamsFinal4[40] = new CourseParameter(  100,    0,    0,    3,   2,  -30,    0,    0,  110 );
        courseParamsFinal4[41] = new CourseParameter(    0,    0,    0,    3,   2,  -30,    0,    0,  110 );
        courseParamsFinal4[42] = new CourseParameter(    0,    0,    0,    3,   0,   10,    0,    0,    0 );
        courseParamsFinal4[43] = new CourseParameter(  100,    0,    0,    0,   2,   15,    0,    0,    0 );
        courseParamsFinal4[44] = new CourseParameter(    0,  250,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal4[45] = new CourseParameter(    0,    0,    0,    2,   2,   70,    0,    1,    0 );
        courseParamsFinal4[46] = new CourseParameter(  400,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsFinal4[47] = new CourseParameter(    0, 2500,    0,    0,   0,    0,    0,    0,    0 );
        courseParamsFinal4[48] = new CourseParameter(    0,    0,    0,    0,  -1,    0,    0,    0,    0 );

        CourseParameter[] courseParamsFinal5 = new CourseParameter[49];
        //                                         ( 距離, 時間,  ｿﾅｰ,   色, ﾄﾚﾓ, 前進, 旋回,  ｱｰﾑ,  PID );
        courseParamsFinal5[0]  = new CourseParameter(    0,   70,    0,    0,   0,   10,  100,    1,   38 );
        courseParamsFinal5[1]  = new CourseParameter(  580,    0,    0,    0,   2,   50,    0,    1,   38 );
        courseParamsFinal5[2]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   38 );
        courseParamsFinal5[3]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsFinal5[4]  = new CourseParameter( 1180,    0,    0,    0,   2,   70,    0,    1,  -17 );
        courseParamsFinal5[5]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsFinal5[6]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal5[7]  = new CourseParameter( 1480,    0,    0,    0,   2,  -70,    0,    1,   -1 );
        courseParamsFinal5[8]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal5[9]  = new CourseParameter(    0,  300,    0,    0,   0,    0,    0,    1,   -1 );
        courseParamsFinal5[10] = new CourseParameter(  240,    0,    0,    0,   2,   30,    0,    1,  -25 );
        courseParamsFinal5[11] = new CourseParameter(  100,    0,    0,    0,   2,   30,    0,    1,    1 );
        courseParamsFinal5[12] = new CourseParameter( 1300,    0,    0,    0,   2,   70,    0,    1,    1 );
        courseParamsFinal5[13] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,    1 );
        courseParamsFinal5[14] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal5[15] = new CourseParameter( 1550,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsFinal5[16] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal5[17] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsFinal5[18] = new CourseParameter(  450,    0,    0,    0,   2,   70,    0,    1,  -44 );
        courseParamsFinal5[19] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsFinal5[20] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        courseParamsFinal5[21] = new CourseParameter( 1200,    0,    0,    0,   2,   70,    0,    1,   19 );
        courseParamsFinal5[22] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        // ここから残りの2本
        courseParamsFinal5[23] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal5[24] = new CourseParameter( 1050,    0,    0,    0,   2,  -70,    0,    1,    0 );
        // 一本目サーチ
        courseParamsFinal5[25] = new CourseParameter(    0,    0,   14,    0,   4,   50,    0,    1,    0 );
        courseParamsFinal5[26] = new CourseParameter(  150,    0,    0,    0,   0,   25,    0,    1,    0 );
        courseParamsFinal5[27] = new CourseParameter(    0, 2500,    0,    0,   0,    3,    0,    0,    0 );
        courseParamsFinal5[28] = new CourseParameter(  100,    0,    0,    3,   2,  -30,    0,    0, -110 );
        courseParamsFinal5[29] = new CourseParameter(    0,    0,    0,    3,   2,  -30,    0,    0, -110 );
        courseParamsFinal5[30] = new CourseParameter(    0,    0,    0,    3,   0,   10,    0,    0,    0 );
        courseParamsFinal5[31] = new CourseParameter(  100,    0,    0,    0,   2,   15,    0,    0,    0 );
        courseParamsFinal5[32] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal5[33] = new CourseParameter(    0,    0,    0,    2,   2,   70,    0,    1,    0 );
        courseParamsFinal5[34] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal5[35] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   17 );
        courseParamsFinal5[36] = new CourseParameter( 1020,    0,    0,    0,   2,  -70,    0,    1,   17 );
        // 二本目サーチ
        courseParamsFinal5[37] = new CourseParameter(    0,    0,   14,    0,   4,   50,    0,    1,    0 );
        courseParamsFinal5[38] = new CourseParameter(  150,    0,    0,    0,   0,   25,    0,    1,    0 );
        courseParamsFinal5[39] = new CourseParameter(    0, 2500,    0,    0,   0,    3,    0,    0,    0 );
        courseParamsFinal5[40] = new CourseParameter(  100,    0,    0,    3,   2,  -30,    0,    0, -110 );
        courseParamsFinal5[41] = new CourseParameter(    0,    0,    0,    3,   2,  -30,    0,    0, -110 );
        courseParamsFinal5[42] = new CourseParameter(    0,    0,    0,    3,   0,   10,    0,    0,    0 );
        courseParamsFinal5[43] = new CourseParameter(  100,    0,    0,    0,   2,   15,    0,    0,    0 );
        courseParamsFinal5[44] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal5[45] = new CourseParameter(    0,    0,    0,    2,   2,   70,    0,    1,    0 );
        courseParamsFinal5[46] = new CourseParameter(  400,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsFinal5[47] = new CourseParameter(    0, 2500,    0,    0,   0,    0,    0,    0,    0 );
        courseParamsFinal5[48] = new CourseParameter(    0,    0,    0,    0,  -1,    0,    0,    0,    0 );


        CourseParameter[] courseParamsFinal6 = new CourseParameter[49];
        //                                         ( 距離, 時間,  ｿﾅｰ,   色, ﾄﾚﾓ, 前進, 旋回,  ｱｰﾑ,  PID );
        courseParamsFinal6[0]  = new CourseParameter(    0,   70,    0,    0,   0,   10,  100,    1,   38 );
        courseParamsFinal6[1]  = new CourseParameter(  580,    0,    0,    0,   2,   50,    0,    1,   38 );
        courseParamsFinal6[2]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   38 );
        courseParamsFinal6[3]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsFinal6[4]  = new CourseParameter( 1180,    0,    0,    0,   2,   70,    0,    1,  -17 );
        courseParamsFinal6[5]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsFinal6[6]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal6[7]  = new CourseParameter( 1480,    0,    0,    0,   2,  -70,    0,    1,   -1 );
        courseParamsFinal6[8]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal6[9]  = new CourseParameter(    0,  300,    0,    0,   0,    0,    0,    1,   -1 );
        courseParamsFinal6[10] = new CourseParameter(  240,    0,    0,    0,   2,   30,    0,    1,  -25 );
        courseParamsFinal6[11] = new CourseParameter(  100,    0,    0,    0,   2,   30,    0,    1,    1 );
        courseParamsFinal6[12] = new CourseParameter( 1300,    0,    0,    0,   2,   70,    0,    1,    1 );
        courseParamsFinal6[13] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,    1 );
        courseParamsFinal6[14] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal6[15] = new CourseParameter( 1550,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsFinal6[16] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal6[17] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsFinal6[18] = new CourseParameter(  450,    0,    0,    0,   2,   70,    0,    1,  -44 );
        courseParamsFinal6[19] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsFinal6[20] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        courseParamsFinal6[21] = new CourseParameter( 1200,    0,    0,    0,   2,   70,    0,    1,   19 );
        courseParamsFinal6[22] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        // ここから残りの2本
        courseParamsFinal6[23] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal6[24] = new CourseParameter( 1050,    0,    0,    0,   2,  -70,    0,    1,    0 );
        // 一本目サーチ
        courseParamsFinal6[25] = new CourseParameter(    0,    0,   14,    0,   3,   50,    0,    1,    0 );
        courseParamsFinal6[26] = new CourseParameter(  150,    0,    0,    0,   0,   25,    0,    1,    0 );
        courseParamsFinal6[27] = new CourseParameter(    0, 2500,    0,    0,   0,    3,    0,    0,    0 );
        courseParamsFinal6[28] = new CourseParameter(  100,    0,    0,    3,   2,  -30,    0,    0,  110 );
        courseParamsFinal6[29] = new CourseParameter(    0,    0,    0,    3,   2,  -30,    0,    0,  110 );
        courseParamsFinal6[30] = new CourseParameter(    0,    0,    0,    3,   0,   10,    0,    0,    0 );
        courseParamsFinal6[31] = new CourseParameter(  100,    0,    0,    0,   2,   15,    0,    0,    0 );
        courseParamsFinal6[32] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal6[33] = new CourseParameter(    0,    0,    0,    2,   2,   70,    0,    1,    0 );
        courseParamsFinal6[34] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal6[35] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal6[36] = new CourseParameter( 1000,    0,    0,    0,   2,  -70,    0,    1,    0 );
        // 二本目サーチ
        courseParamsFinal6[37] = new CourseParameter(    0,    0,   14,    0,   4,   50,    0,    1,    0 );
        courseParamsFinal6[38] = new CourseParameter(  150,    0,    0,    0,   0,   25,    0,    1,    0 );
        courseParamsFinal6[39] = new CourseParameter(    0, 2500,    0,    0,   0,    3,    0,    0,    0 );
        courseParamsFinal6[40] = new CourseParameter(  100,    0,    0,    3,   2,  -30,    0,    0, -110 );
        courseParamsFinal6[41] = new CourseParameter(    0,    0,    0,    3,   2,  -30,    0,    0, -110 );
        courseParamsFinal6[42] = new CourseParameter(    0,    0,    0,    3,   0,   10,    0,    0,    0 );
        courseParamsFinal6[43] = new CourseParameter(  100,    0,    0,    0,   2,   15,    0,    0,    0 );
        courseParamsFinal6[44] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal6[45] = new CourseParameter(    0,    0,    0,    2,   2,   70,    0,    1,    0 );
        courseParamsFinal6[46] = new CourseParameter(  400,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsFinal6[47] = new CourseParameter(    0, 2500,    0,    0,   0,    0,    0,    0,    0 );
        courseParamsFinal6[48] = new CourseParameter(    0,    0,    0,    0,  -1,    0,    0,    0,    0 );

        CourseParameter[] courseParamsFinal7 = new CourseParameter[49];
        //                                         ( 距離, 時間,  ｿﾅｰ,   色, ﾄﾚﾓ, 前進, 旋回,  ｱｰﾑ,  PID );
        courseParamsFinal7[0]  = new CourseParameter(    0,   70,    0,    0,   0,   10,  100,    1,   38 );
        courseParamsFinal7[1]  = new CourseParameter(  580,    0,    0,    0,   2,   50,    0,    1,   38 );
        courseParamsFinal7[2]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   38 );
        courseParamsFinal7[3]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsFinal7[4]  = new CourseParameter( 1180,    0,    0,    0,   2,   70,    0,    1,  -17 );
        courseParamsFinal7[5]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsFinal7[6]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal7[7]  = new CourseParameter( 1480,    0,    0,    0,   2,  -70,    0,    1,   -1 );
        courseParamsFinal7[8]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal7[9]  = new CourseParameter(    0,  300,    0,    0,   0,    0,    0,    1,   -1 );
        courseParamsFinal7[10] = new CourseParameter(  240,    0,    0,    0,   2,   30,    0,    1,  -25 );
        courseParamsFinal7[11] = new CourseParameter(  100,    0,    0,    0,   2,   30,    0,    1,    1 );
        courseParamsFinal7[12] = new CourseParameter( 1300,    0,    0,    0,   2,   70,    0,    1,    1 );
        courseParamsFinal7[13] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,    1 );
        courseParamsFinal7[14] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal7[15] = new CourseParameter( 1550,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsFinal7[16] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal7[17] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsFinal7[18] = new CourseParameter(  450,    0,    0,    0,   2,   70,    0,    1,  -44 );
        courseParamsFinal7[19] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsFinal7[20] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        courseParamsFinal7[21] = new CourseParameter( 1200,    0,    0,    0,   2,   70,    0,    1,   19 );
        courseParamsFinal7[22] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        // ここから残りの2本
        courseParamsFinal7[23] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,  -17 );
        courseParamsFinal7[24] = new CourseParameter( 1120,    0,    0,    0,   2,  -70,    0,    1,  -17 );
        // 一本目サーチ
        courseParamsFinal7[25] = new CourseParameter(    0,    0,   14,    0,   3,   50,    0,    1,    0 );
        courseParamsFinal7[26] = new CourseParameter(  150,    0,    0,    0,   0,   25,    0,    1,    0 );
        courseParamsFinal7[27] = new CourseParameter(    0, 2500,    0,    0,   0,    3,    0,    0,    0 );
        courseParamsFinal7[28] = new CourseParameter(  100,    0,    0,    3,   2,  -30,    0,    0,  110 );
        courseParamsFinal7[29] = new CourseParameter(    0,    0,    0,    3,   2,  -30,    0,    0,  110 );
        courseParamsFinal7[30] = new CourseParameter(    0,    0,    0,    3,   0,   10,    0,    0,    0 );
        courseParamsFinal7[31] = new CourseParameter(  100,    0,    0,    0,   2,   15,    0,    0,    0 );
        courseParamsFinal7[32] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal7[33] = new CourseParameter(    0,    0,    0,    2,   2,   70,    0,    1,    0 );
        courseParamsFinal7[34] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal7[35] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal7[36] = new CourseParameter( 1000,    0,    0,    0,   2,  -70,    0,    1,    0 );
        // 二本目サーチ
        courseParamsFinal7[37] = new CourseParameter(    0,    0,   14,    0,   4,   50,    0,    1,    0 );
        courseParamsFinal7[38] = new CourseParameter(  150,    0,    0,    0,   0,   25,    0,    1,    0 );
        courseParamsFinal7[39] = new CourseParameter(    0, 2500,    0,    0,   0,    3,    0,    0,    0 );
        courseParamsFinal7[40] = new CourseParameter(  100,    0,    0,    3,   2,  -30,    0,    0, -110 );
        courseParamsFinal7[41] = new CourseParameter(    0,    0,    0,    3,   2,  -30,    0,    0, -110 );
        courseParamsFinal7[42] = new CourseParameter(    0,    0,    0,    3,   0,   10,    0,    0,    0 );
        courseParamsFinal7[43] = new CourseParameter(  100,    0,    0,    0,   2,   15,    0,    0,    0 );
        courseParamsFinal7[44] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal7[45] = new CourseParameter(    0,    0,    0,    2,   2,   70,    0,    1,    0 );
        courseParamsFinal7[46] = new CourseParameter(  400,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsFinal7[47] = new CourseParameter(    0, 2500,    0,    0,   0,    0,    0,    0,    0 );
        courseParamsFinal7[48] = new CourseParameter(    0,    0,    0,    0,  -1,    0,    0,    0,    0 );


        CourseParameter[] courseParamsFinal8 = new CourseParameter[49];
        //                                         ( 距離, 時間,  ｿﾅｰ,   色, ﾄﾚﾓ, 前進, 旋回,  ｱｰﾑ,  PID );
        courseParamsFinal8[0]  = new CourseParameter(    0,   70,    0,    0,   0,   10,  100,    1,   38 );
        courseParamsFinal8[1]  = new CourseParameter(  580,    0,    0,    0,   2,   50,    0,    1,   38 );
        courseParamsFinal8[2]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   38 );
        courseParamsFinal8[3]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsFinal8[4]  = new CourseParameter( 1180,    0,    0,    0,   2,   70,    0,    1,  -17 );
        courseParamsFinal8[5]  = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -17 );
        courseParamsFinal8[6]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal8[7]  = new CourseParameter( 1480,    0,    0,    0,   2,  -70,    0,    1,   -1 );
        courseParamsFinal8[8]  = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,   -1 );
        courseParamsFinal8[9]  = new CourseParameter(    0,  300,    0,    0,   0,    0,    0,    1,   -1 );
        courseParamsFinal8[10] = new CourseParameter(  240,    0,    0,    0,   2,   30,    0,    1,  -25 );
        courseParamsFinal8[11] = new CourseParameter(  100,    0,    0,    0,   2,   30,    0,    1,    1 );
        courseParamsFinal8[12] = new CourseParameter( 1300,    0,    0,    0,   2,   70,    0,    1,    1 );
        courseParamsFinal8[13] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,    1 );
        courseParamsFinal8[14] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal8[15] = new CourseParameter( 1550,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsFinal8[16] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal8[17] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsFinal8[18] = new CourseParameter(  450,    0,    0,    0,   2,   70,    0,    1,  -44 );
        courseParamsFinal8[19] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,  -44 );
        courseParamsFinal8[20] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        courseParamsFinal8[21] = new CourseParameter( 1200,    0,    0,    0,   2,   70,    0,    1,   19 );
        courseParamsFinal8[22] = new CourseParameter(  100,    0,    0,    0,   2,   20,    0,    1,   19 );
        // ここから残りの2本
        courseParamsFinal8[23] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,    0 );
        courseParamsFinal8[24] = new CourseParameter( 1050,    0,    0,    0,   2,  -70,    0,    1,    0 );
        // 一本目サーチ
        courseParamsFinal8[25] = new CourseParameter(    0,    0,   14,    0,   4,   50,    0,    1,    0 );
        courseParamsFinal8[26] = new CourseParameter(  150,    0,    0,    0,   0,   25,    0,    1,    0 );
        courseParamsFinal8[27] = new CourseParameter(    0, 2500,    0,    0,   0,    3,    0,    0,    0 );
        courseParamsFinal8[28] = new CourseParameter(  100,    0,    0,    3,   2,  -30,    0,    0,  110 );
        courseParamsFinal8[29] = new CourseParameter(    0,    0,    0,    3,   2,  -30,    0,    0,  110 );
        courseParamsFinal8[30] = new CourseParameter(    0,    0,    0,    3,   0,   10,    0,    0,    0 );
        courseParamsFinal8[31] = new CourseParameter(  100,    0,    0,    0,   2,   15,    0,    0,    0 );
        courseParamsFinal8[32] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal8[33] = new CourseParameter(    0,    0,    0,    2,   2,   70,    0,    1,    0 );
        courseParamsFinal8[34] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal8[35] = new CourseParameter(  100,    0,    0,    0,   2,  -20,    0,    1,  -17 );
        courseParamsFinal8[36] = new CourseParameter( 1020,    0,    0,    0,   2,  -70,    0,    1,  -17 );
        // 二本目サーチ
        courseParamsFinal8[37] = new CourseParameter(    0,    0,   14,    0,   3,   50,    0,    1,    0 );
        courseParamsFinal8[38] = new CourseParameter(  150,    0,    0,    0,   0,   25,    0,    1,    0 );
        courseParamsFinal8[39] = new CourseParameter(    0, 2500,    0,    0,   0,    3,    0,    0,    0 );
        courseParamsFinal8[40] = new CourseParameter(  100,    0,    0,    3,   2,  -30,    0,    0, -110 );
        courseParamsFinal8[41] = new CourseParameter(    0,    0,    0,    3,   2,  -30,    0,    0, -110 );
        courseParamsFinal8[42] = new CourseParameter(    0,    0,    0,    3,   0,   10,    0,    0,    0 );
        courseParamsFinal8[43] = new CourseParameter(  100,    0,    0,    0,   2,   15,    0,    0,    0 );
        courseParamsFinal8[44] = new CourseParameter(    0,  750,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsFinal8[45] = new CourseParameter(    0,    0,    0,    2,   2,   70,    0,    1,    0 );
        courseParamsFinal8[46] = new CourseParameter(  400,    0,    0,    0,   2,  -70,    0,    1,    0 );
        courseParamsFinal8[47] = new CourseParameter(    0, 2500,    0,    0,   0,    0,    0,    0,    0 );
        courseParamsFinal8[48] = new CourseParameter(    0,    0,    0,    0,  -1,    0,    0,    0,    0 );


        CourseParameter[] courseParamsArmTest = new CourseParameter[10];
        //                                           ( 距離, 時間,  ｿﾅｰ,   色, ﾄﾚﾓ, 前進, 旋回,  ｱｰﾑ,  PID );
        courseParamsArmTest[0]  = new CourseParameter(    0,    0,   12,    0,   3,   50,    0,    1,    0 );
        courseParamsArmTest[1]  = new CourseParameter(  130,    0,    0,    0,   0,   25,    0,    1,    0 );
        courseParamsArmTest[2]  = new CourseParameter(    0, 2500,    0,    0,   0,    3,    0,    0,    0 );
        courseParamsArmTest[3]  = new CourseParameter(   50,    0,    0,    0,   2,  -23,    0,    0,    0 );
        courseParamsArmTest[4]  = new CourseParameter( 1000,    0,    0,    0,   2,  -23,    0,    0,    0 );
        courseParamsArmTest[5]  = new CourseParameter(  200,    0,    0,    0,   2,   10,    0,    0,    0 );
        courseParamsArmTest[6]  = new CourseParameter(    0,    0,    0,    2,   2,   50,    0,    0,    0 );
        courseParamsArmTest[7]  = new CourseParameter(  200,    0,    0,    0,   2,   10,    0,    0,    0 );
        courseParamsArmTest[8]  = new CourseParameter(    0,    0,    0,    5,   2,  -50,    0,    0,  180 );
        courseParamsArmTest[9]  = new CourseParameter(    0,    0,    0,    0,  -1,    0,    0,    0,    0 );

        CourseParameter[] courseParamsLineTest = new CourseParameter[6];
        //                                            ( 距離, 時間,  ｿﾅｰ,   色, ﾄﾚﾓ, 前進, 旋回,  ｱｰﾑ,  PID );
        courseParamsLineTest[0]  = new CourseParameter(    0, 5000,    0,    0,   0,    0,    0,    0,    0 );
        courseParamsLineTest[1]  = new CourseParameter(    0, 5000,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsLineTest[2]  = new CourseParameter(    0, 5000,    0,    0,   0,    0,    0,    0,    0 );
        courseParamsLineTest[3]  = new CourseParameter(    0, 5000,    0,    0,   0,    0,    0,    1,    0 );
        courseParamsLineTest[4]  = new CourseParameter(    0, 5000,    0,    0,   0,    0,    0,    0,    0 );
        courseParamsLineTest[5]  = new CourseParameter(    0,    0,    0,    0,  -1,    0,    0,    0,    0 );



        selectCourse.add(courseParamsQ);
        selectCourse.add(courseParamsFinal1);
        selectCourse.add(courseParamsFinal2);
        selectCourse.add(courseParamsFinal3);
        selectCourse.add(courseParamsFinal4);
        selectCourse.add(courseParamsFinal5);
        selectCourse.add(courseParamsFinal6);
        selectCourse.add(courseParamsFinal7);
        selectCourse.add(courseParamsFinal8);
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
        int beforeDistance = 0;
        int tmptmpGyro = 0;
        int tmpGyro = 0;
        int gyroAngle = 0;
        int search = 0;
        int sonerSearchS = 0;
        int selectCourseNumbar = 0;
        int reverseSwitch = 0;
        int sonerTmp = 0;
        int runningTime = 62000;
        int forwardT = 5;
        int turnT = 0;

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

            tmptmpGyro = tmpGyro = gyroInt;


            // 色の判定
            colorId = col.decision(red, green, blue);


            // LCD出力
            LCD.clear();
            LCD.drawString("Ready?  " + selectCourseNumbar, 0, 0);
            LCD.drawString("Red: " + red, 0, 1);
            LCD.drawString("Gre: " + green, 0, 2);
            LCD.drawString("Blu: " + blue, 0, 3);
            LCD.drawString("RGB: " + sonicInt, 0, 4);
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
            runningTime = 122000;
        }

        // アームモーターリセット
        armMotorInit();

        // タイマー計測開始
        counter.reset();
        stopwatch.reset();
        if (reverseSwitch == 1) {
            sonerSearchS = 1;

        }
        sonerTmp = 105;

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

            if (touchInt == 1) {
                sonicInt = 1;
            }

            // 色の判定
            colorId = col.decision(red, green, blue);

            // 区間情報変更の確認
            if (courseChange(courseParams[courseNumber], dis.getDistance() - beforeDistance, stopwatch.elapsed(), sonicInt, colorId)) {
                courseNumber++;
                beforeDistance = dis.getDistance();
                stopwatch.reset();
                //ビープ音を鳴らす
                Sound.beep();
                search = 0;
                tmptmpGyro = tmpGyro = gyroInt;
                sonerTmp = 105;
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
                if (search == 0 || search == 1) {
                    if (search == 0) {
                        gyroAngle = 200;
                        forwardT = 4;
                        turnT = 100;
                    }
                    else if (search == 1) {
                        gyroAngle = 30;
                        forwardT = -2;
                        turnT = -40;
                    }
                    if (gyroAngle > gyroInt - tmpGyro && sonerSearchS == 0) {
                        turn = turnT;
                        forward = forwardT;
                    }
                    else if (sonerSearchS == 0){
                        sonerSearchS = 1;
                    }

                    if (-gyroAngle < gyroInt - tmpGyro && sonerSearchS == 1) {
                        turn = -turnT;
                        forward = forwardT;
                    }
                    else if (sonerSearchS == 1){
                        sonerSearchS = 0;
                    }

                    if (sonicInt < sonerTmp) {
                        if (search == 0) {
                            tmpGyro = gyroInt;
                        }
                        search = 2;
                        sonerTmp = sonicInt + 25;
                        if (sonerTmp > 105) {
                            sonerTmp = 105;
                        }
                    }
                }
                else if (search == 2) {
                    if (sonicInt <= sonerTmp) {
                        sonerTmp = sonicInt + 25;
                        if (sonerTmp > 105) {
                            sonerTmp = 105;
                            search = 0;
                            tmpGyro = tmptmpGyro;
                        }
                    }
                    else {
                        search = 1;
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

            armAngle = armMotor.getTachoCount();

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
        int pwm = 0;
        int armTarget = 0;
        if (armMode == 0) {
            armTarget = -95;
        }
        if (armMode == 1) {
            armTarget = 600;
        }
        pwm = armPID.calcControl(armTarget - armAngle);
        armMotorSet(pwm);
    }
}
