package org.firstinspires.ftc.teamcode.Recode.Hardware;

import static org.firstinspires.ftc.teamcode.Constants.Color.Green;
import static org.firstinspires.ftc.teamcode.Constants.Color.None;
import static org.firstinspires.ftc.teamcode.Constants.Color.Purple;
import static org.firstinspires.ftc.teamcode.Constants.GreenValuesHSV;
import static org.firstinspires.ftc.teamcode.Constants.PurpleValuesHSV;
import static org.firstinspires.ftc.teamcode.Constants.Treshold;
import static org.firstinspires.ftc.teamcode.Constants.currentColor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Algorithms.LowPassFilter;

import java.util.Arrays;

@SuppressWarnings("All")

@Config
public class ArtifactSensor {

    ColorRangeSensor sensor;

    private final LowPassFilter redFilter, blueFilter, greenFilter;

    double filterTradeOff = 0.9;

    int[] rgbValues=new int[4];
    float[] hsvValues = new float[4];

    public ArtifactSensor(HardwareMap hardwareMap, String name)
    {
        sensor = hardwareMap.get(ColorRangeSensor.class, name);

        redFilter = new LowPassFilter(filterTradeOff, sensor.red());
        greenFilter = new LowPassFilter(filterTradeOff, sensor.green());
        blueFilter = new LowPassFilter(filterTradeOff, sensor.blue());
    }

    public double getDistance(DistanceUnit unit)
    {
        return sensor.getDistance(unit);
    }

    public double getFilterTradeOff()
    {
        return filterTradeOff;
    }

    public Constants.Color getColor()
    {
        return currentColor;
    }

    public int[] getRgbValues()
    {
        return rgbValues;
    }

    public float[] getHsvValues()
    {
        return hsvValues;
    }

    public void update()
    {
        rgbValues[0]=(int) redFilter.getValue(sensor.red());
        rgbValues[1]=(int) greenFilter.getValue(sensor.green());
        rgbValues[2]=(int) blueFilter.getValue(sensor.blue());

        android.graphics.Color.RGBToHSV(rgbValues[0] * 8, rgbValues[1] * 8, rgbValues[2] * 8, hsvValues);

        if (Math.abs(hsvValues[0] - GreenValuesHSV[0]) <= Treshold[0]) {
            currentColor = Green;
        } else if (Math.abs(hsvValues[0] - PurpleValuesHSV[0]) <= Treshold[0]) {
            currentColor = Purple;
        } else {
            currentColor = None;
        }

    }

    public void telemetry(Telemetry telemetry)
    {
        telemetry.addData("Color in RGB", Arrays.toString(this.rgbValues));
        telemetry.addData("Color in HSV", Arrays.toString(this.hsvValues));
        telemetry.addData("currentColor", currentColor);
        telemetry.update();
    }

}
