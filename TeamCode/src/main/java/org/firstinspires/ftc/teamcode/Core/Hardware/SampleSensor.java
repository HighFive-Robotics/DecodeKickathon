package org.firstinspires.ftc.teamcode.Core.Hardware;





import static org.firstinspires.ftc.teamcode.Constants.Color.None;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.Arrays;

//todo rework this please
@Config
public class SampleSensor {
    ColorRangeSensor sensor;
    private final LowPassFilter redFilter, blueFilter, greenFilter;
    double filterParameter = 0.8;
    private int r, g, b;
    float[] hsvValues = new float[4];

    Constants.Color color = None;

    public SampleSensor(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.get(ColorRangeSensor.class, name);

        redFilter = new LowPassFilter(filterParameter, sensor.red());
        greenFilter = new LowPassFilter(filterParameter, sensor.green());
        blueFilter = new LowPassFilter(filterParameter, sensor.blue());
    }

    public double getDistance(DistanceUnit distanceUnit) {
        return sensor.getDistance(distanceUnit);
    }

    public void update() {
        r = (int) redFilter.getValue(sensor.red());
        g = (int) greenFilter.getValue(sensor.green());
        b = (int) blueFilter.getValue(sensor.blue());

        android.graphics.Color.RGBToHSV(r * 8, g * 8, b * 8, hsvValues);
        if (Math.abs(hsvValues[0] - YellowValues[0]) <= TreshHold[0]) {
            color = Yellow;
        } else if (Math.abs(hsvValues[0] - BlueValues[0]) <= TreshHold[0]) {
            color = Blue;

        } else if (Math.abs(hsvValues[0] - RedValues[0]) <= TreshHold[0]) {
            color = Red;
        } else {
            color = None;
        }
    }

    public Constants.Color getColor() {
        return color;
    }
    public void telemetry(Telemetry telemetry){
        telemetry.addData("Color in RGB", Arrays.toString(this.RGB()));
        telemetry.addData("Color in HSV" ,Arrays.toString(this.hsvValues));
        telemetry.addData("Color known", this.getColor());
    }

    public double[] RGB() {
        return new double[]{r, g, b};
    }
}