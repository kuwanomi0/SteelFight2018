package steelfight;

public class CourseParameter {
    private int dis;        /* 設定距離 */
    private int time;       /* 設定時間 */
    private int sonarDis;   /* 障害物検知距離 */
    private int colorId;    /* 色 */
    private int traceMode;  /* トレースモード (0:トレースなし  1:ライントレース  2:ジャイロトレース 3:soner) */
    private int forward;    /* 前進速度 */
    private int turn;       /* 旋回速度 */
    private int armMode;    /* アーム状態設定 (0: 開く  1:閉じる) */
    private int pidTarget;  /* PID目標値 */

    public CourseParameter(int dis, int time, int sonarDis, int colorId, int traceMode, int forward, int turn, int armMode, int pidTarget) {
        super();
        this.dis = dis;
        this.time = time;
        this.sonarDis = sonarDis;
        this.colorId = colorId;
        this.traceMode = traceMode;
        this.forward = forward;
        this.turn = turn;
        this.armMode = armMode;
        this.pidTarget = pidTarget;
    }

    public int getColorId() {
        return colorId;
    }

    public int getPidTarget() {
        return pidTarget;
    }

    public int getDis() {
        return dis;
    }


    public int getTime() {
        return time;
    }


    public int getSonarDis() {
        return sonarDis;
    }


    public int getTraceMode() {
        return traceMode;
    }


    public int getForward() {
        return forward;
    }


    public int getTurn() {
        return turn;
    }


    public int getArmMode() {
        return armMode;
    }


}
