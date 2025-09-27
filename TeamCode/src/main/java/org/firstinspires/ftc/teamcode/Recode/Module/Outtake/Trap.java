package org.firstinspires.ftc.teamcode.Recode.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.trapSensorName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.trapServoName;
import static org.firstinspires.ftc.teamcode.Constants.currentColor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.ArtifactSensor;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

@Config
public class Trap implements HighModule {
    public HighServo trapServo;
    public ArtifactSensor sensor;
    public static double openPose = 0.55, closedPose = 0.8;
    public static Constants.Color wantedColor = Constants.Color.Blue;
    public States state= States.None;
    public enum States
    {
        Closed,
        Open,
        None
    }
    private double target;

    public Trap(HardwareMap hardwareMap, double initPos, boolean isAuto)
    {
        trapServo=new HighServo(hardwareMap.get(Servo.class, trapServoName), HighServo.RunMode.Standard, initPos, isAuto);
        sensor=new ArtifactSensor(hardwareMap, trapSensorName);
        target=initPos;
    }

    public void setState(States state, double time)
    {
        this.state = state;
        switch (state)
        {
            case Closed:
                if(time == 0)setTarget(closedPose);
                else setTarget(closedPose, time);
                break;
            case Open:
                if(time == 0)setTarget(openPose);
                else setTarget(openPose, time);
                break;
        }
    }
    @Override
    public void setTarget(double target)
    {
        this.target = target;
        trapServo.setPosition(target);
    }
    @Override
    public void setTarget(double target, double time)
    {
        this.target = target;
        trapServo.setPosition(target, time);
    }
    @Override
    public boolean atTarget() {return trapServo.atTarget();}
    @Override
    public double getTarget() {return target;}
    public void setWantedColor(Constants.Color color){
        wantedColor = color;
    }
    @Override
    public void update()
    {
        if(wantedColor == currentColor && state != States.Open)
        {
            setState(States.Open, trapServo.time);
        }
        trapServo.update();
        sensor.update();
    }
}
