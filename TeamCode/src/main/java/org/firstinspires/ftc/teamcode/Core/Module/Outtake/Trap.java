package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

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

    public static double openPose = 0, closedPose = 1;

    HighServo trapServo;
    ArtifactSensor sensor;
    Constants.Color wantedColor = Constants.Color.Blue;
    public States state = States.None;
    private double target;

    public enum States {
        Open,
        Closed,
        None
    }

    public Trap(HardwareMap hardwareMap, double initPosition, boolean isAuto){
        trapServo = new HighServo(hardwareMap.get(Servo.class, trapServoName) ,HighServo.RunMode.Standard, initPosition ,isAuto);
        target = initPosition;
    }

    public void setState(States state) {
        this.state = state;
        switch (state) {
            case Open:
                setTarget(openPose);
                break;
            case Closed:
                setTarget(closedPose);
                break;
        }
    }

    public void setState(States state, double time) {
        this.state = state;
        switch (state) {
            case Open:
                setTarget(openPose, time);
                break;
            case Closed:
                setTarget(closedPose, time);
                break;
        }
    }

    @Override
    public void setTarget(double target) {
        this.target = target;
        trapServo.setPosition(target);
    }

    @Override
    public void setTarget(double target, double time) {
        this.target = target;
        trapServo.setPosition(target, time);
    }

    @Override
    public boolean atTarget() {
        return trapServo.atTarget();
    }

    @Override
    public double getTarget() {
        return target;
    }

    public States getState() {
        return state;
    }

    public void setWantedColor(Constants.Color color){
        wantedColor = color;
    }

    @Override
    public void update() {
        if(wantedColor == currentColor && state != States.Open){
            setState(States.Open);
        }
        else{
            setState(States.Closed);
        }
        trapServo.update();
        sensor.update();
    }
}
