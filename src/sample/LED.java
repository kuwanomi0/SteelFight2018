package sample;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;

public class LED {
    public static void main(String[] args) {
        // ---- LED点灯
        // LEDPattern には 0, 1, 2, 3, 4, 5, 6 の状態
        for ( int k = 0; k <= 6; k++) {
            String str = String.valueOf(k);
            LCD.clear();
            LCD.drawString(str, 0, 5);
            Button.LEDPattern(k);
            Delay.msDelay(3000);
        }
    }
}

/*
それぞれの色

0 消灯
1 緑
2 赤
3 橙
4 緑 点滅
5 赤 点滅
6 橙 点滅

*/