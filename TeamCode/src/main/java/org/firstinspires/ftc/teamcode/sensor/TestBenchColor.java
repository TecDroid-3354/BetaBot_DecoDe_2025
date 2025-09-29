package org.firstinspires.ftc.teamcode.sensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.hardware.SensorColor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/* To connect to the robot and deploy the code wirelessly, type the following in the terminal:
 *    adb connect 192.168.43.1:5555 (connects to Control Hub)
 *    adb connect 192.168.43.1:8080 (connects to FTC Dashboard)
 * To visit the FTC dashboard online (while connected to the Control Hub's internet)
 *    http://192.168.43.1:8080/?page=connection.html&pop=true
 */

public class TestBenchColor {
    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    public void init(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

    }

    public DetectedColor getDetectedColor(Telemetry telemetry) {
        // Returns 4 values: red, green, blue, and alpha
        // Each of the colors represent their respective color values, except for alpha,
        // which represents the brightness of the color

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // We normalize the colors once again to mitigate error as much as possible: this way,
        // no matter what light value or distance it is, we should be able to get the same color
        // every time
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry.addData("Red", normRed);
        telemetry.addData("Green", normGreen);
        telemetry.addData("Blue", normBlue);

        return DetectedColor.UNKNOWN;
    }
}