package steelfight;

import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class SecondCounter {
    private int second;

    // 1000ミリ秒ごとに割り込みを発生させる
    private Timer timer = new Timer(1000, new CountTimer());

    public void reset() {
        second = 0;
    }

    public int getSecond() {
        return second;
    }

    public void start() {
        timer.start();
    }

    public void stop() {
        timer.stop();
    }

    class CountTimer implements TimerListener {
        // 1000ミリ秒ごとにtimedOut()メソッドが、Timerから呼ばれる
        public void timedOut() {
            second++;
        }
    }
}