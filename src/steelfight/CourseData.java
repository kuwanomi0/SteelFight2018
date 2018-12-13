package steelfight;

public class CourseData {
    private int dis;        /* 設定距離 */
    private int time;       /* 設定時間 */
//    int sonarDis;   /* 障害物検知距離 */
//    int mode;     /* トレースモード */
    int forward;    /* 前進速度 */
    int turn;       /* 旋回速度 */
//    int arm;  /* 尻尾角度 */
//    int gyroOffset; /* ジャイロオフセット */
//    float kp;       /* 係数P */
//    float ki;       /* 係数I */
//    float kd;       /* 係数D */
//    float krgb;     /* RGB目標値係数 */

    public CourseData(int dis, int time, int forward,int turn) {
        this.dis = dis;
        this.time = time;
        this.forward = forward;
        this.turn = turn;
    }

    public int getDis() {
        return dis;
    }

    public int getTime() {
        return time;
    }

    public int getForward() {
        return forward;
    }

    public int getTurn() {
        return turn;
    }


}
