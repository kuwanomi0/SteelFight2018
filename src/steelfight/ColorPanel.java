package steelfight;

public class ColorPanel {
    int[] black  = {  3,  3,  2 };
    int[] kRed    = { 28,  5,  3 };
    int[] kGreen  = {  5, 12,  5 };
    int[] kBlue   = {  4,  8, 15 };
    int[] yellow = { 34, 15,  3 };
    int[] white  = { 35, 28, 25 };

    public ColorPanel() {
    }

    public int decision(int red, int green, int blue) {
        int colorid = 0;

        // 1 赤  23以上  7未満  7未満
        if (
                (kRed[0] - 5) <= red &&
                (kRed[1] + 2) > green &&
                (kRed[2] + 4) > blue) {
            colorid = 2;
        }

        // 2 緑  7未満  10以上  7未満
        if (
                (kGreen[0] + 2) > red &&
                (kGreen[1] - 2) <= green &&
                (kGreen[2] + 2) > blue) {
            colorid = 2;
        }

        // 3 青  7未満  10未満  11以上
        if (
                (kBlue[0] + 3) > red &&
                (kBlue[1] + 2) > green &&
                (kBlue[2] - 4) <= blue) {
            colorid = 3;
        }

        // 4 黄 29以上  13以上  8未満
        if (
                (yellow[0] - 5) <= red &&
                (yellow[1] - 2) <= green &&
                (yellow[2] + 5) > blue) {
            colorid = 4;
        }

        // 5 白 30以上  23以上  20以上
        if (
                (white[0] - 5) <= red &&
                (white[1] - 5) <= green &&
                (white[2] - 5) <= blue) {
            colorid = 5;
        }

        return colorid;
    }
}
