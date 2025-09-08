package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorName;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModuleSimple;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

public class Shooter implements HighModuleSimple {
    HighMotor shooterMotor;
    private double power = 0;
    ElapsedTime shooterTimer = new ElapsedTime();

    public enum States {
        Stopped,
        ChargingToShoot,
        ReadyToShoot,
        None
    }

    States state = States.None;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = new HighMotor(hardwareMap.get(DcMotorEx.class, shooterMotorName), HighMotor.RunMode.Standard, true, true);
        state = States.Stopped;
    }

    public void stop() {
        state = States.Stopped;
        power = 0;
    }

    public void shoot() {
        state = States.ChargingToShoot;
        power = 1;
        shooterTimer.reset();
    }
    public States getState(){
        return state;
    }

    @Override
    public void update() {
        if (shooterTimer.milliseconds() >= 400 && state == States.ChargingToShoot) {
            state = States.ReadyToShoot;
        }
        shooterMotor.setPower(power);
        shooterMotor.update();
    }
}
