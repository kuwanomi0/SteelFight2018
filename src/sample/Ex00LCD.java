package sample;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;

public class Ex00LCD {
    public static void main(String[] args) {
        LCD.clear();
        LCD.drawString("> Yes     NO", 0, 5);
        Delay.msDelay(5000);
        LCD.drawString("EV3 G13", 0, 5);
        Button.waitForAnyPress();
        LCD.clear();
        LCD.refresh();
    }
}
