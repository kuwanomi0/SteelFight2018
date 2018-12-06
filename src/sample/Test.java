package sample;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;

public class Test {

    public static void main(String[] args) {
        for ( int k = 0; k <= 6; k++) {
            String str = String.valueOf(k);
            LCD.clear();
            LCD.drawString(str, 0, 5);
            Button.LEDPattern(k);
            Delay.msDelay(3000);
        }
    }

}
