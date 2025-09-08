package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.pusherServoName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.trapServoName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.ArtifactSensor;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

@Config
public class Pusher implements HighModule {

    public static double retractedPose = 0.3, extendedPose = 0.5;

    HighServo pushServo;
    public States state = States.None;
    private double target;


    public enum States {
        Retracted,
        Extended,
        None
    }

    public Pusher(HardwareMap hardwareMap, double initPosition, boolean isAuto){
        pushServo = new HighServo(hardwareMap.get(Servo.class, pusherServoName) ,HighServo.RunMode.Standard, initPosition ,isAuto);
        target = initPosition;
    }

    public void setState(States state) {
        this.state = state;
        switch (state) {
            case Retracted:
                setTarget(retractedPose);
                break;
            case Extended:
                setTarget(extendedPose);
                break;
        }
    }

    public void setState(States state, double time) {
        this.state = state;
        switch (state) {
            case Retracted:
                setTarget(retractedPose, time);
                break;
            case Extended:
                setTarget(extendedPose, time);
                break;
        }
    }

    @Override
    public void setTarget(double target) {
        this.target = target;
        pushServo.setPosition(target);
    }

    @Override
    public void setTarget(double target, double time) {
        this.target = target;
        pushServo.setPosition(target, time);
    }

    @Override
    public boolean atTarget() {
        return pushServo.atTarget();
    }

    @Override
    public double getTarget() {
        return target;
    }

    @Override
    public void update() {
        pushServo.update();
    }
}
