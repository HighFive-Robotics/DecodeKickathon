package org.firstinspires.ftc.teamcode.Recode.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.pusherServoName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

@Config
public class Pusher implements HighModule {
    HighServo pusherServo;
    public static double retractedPose = 0.3, extendedPose = 0.5;
    public States state= States.None;
    public enum States
    {
        Retracted,
        Extended,
        None
    }
    private double target;
    public Pusher(HardwareMap hardwareMap, double initPos, boolean isAuto)
    {
        pusherServo= new HighServo(hardwareMap.get(Servo.class,pusherServoName), HighServo.RunMode.Standard,initPos,isAuto);
        target=initPos;

    }
    public void setState(States state, double time)
    {
        this.state = state;
        switch (state)
        {
            case Retracted:
                if (time == 0) setTarget(retractedPose);
                else setTarget(retractedPose, time);
                break;
            case Extended:
                if (time == 0) setTarget(extendedPose);
                else setTarget(extendedPose, time);
                break;
        }
    }

    @Override
    public void setTarget(double target) {
        this.target=target;
        pusherServo.setPosition(target);
    }

    @Override
    public void setTarget(double target, double time) {
        this.target=target;
        pusherServo.setPosition(target,time);
    }

    @Override
    public boolean atTarget() {return pusherServo.atTarget();}

    @Override
    public double getTarget() {return target;}

    @Override
    public void update(){pusherServo.update();}
}
