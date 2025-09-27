package org.firstinspires.ftc.teamcode.Recode.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorName;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModuleSimple;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

@SuppressWarnings("All")
public class Shooter implements HighModuleSimple {
    HighMotor shooterMotor;
    ElapsedTime timer = new ElapsedTime();
    private final double TimeToCharge = 400;
    public enum States{

        Stopped(0),
        Charging(1),
        ReadyToShoot(1),
        None(0);
        public double power;
        States(double power){
            this.power = power;
        }
    }
    States state = States.None;
    double power = getState().power;

    /**
     *
     * @param hardwareMap
     */
    public Shooter(@NonNull HardwareMap hardwareMap){
        shooterMotor = new HighMotor(hardwareMap.get(DcMotorEx.class, shooterMotorName), HighMotor.RunMode.Standard, true, true);
        state = States.Stopped;
    }

    /**
     *
     * @param state
     */
    private void setState(@NonNull States state){
        this.state = state;
        switch (state){
            case None:
                break;
            case Charging:
                power = state.power;
                timer.reset();
                break;
            case ReadyToShoot:
                break;
            case Stopped:
                power = state.power;
                break;
        }
    }
    public void shoot(){
        setState(States.Charging);
    }
    public void stop(){
        setState(States.Stopped);
    }
    public States getState(){
        return this.state;
    }
    public boolean readyToShoot(){
        return state == States.ReadyToShoot;
    }
    @Override
    public void update() {
        if(timer.milliseconds() >= TimeToCharge && getState().equals(States.Charging)){
            setState(States.ReadyToShoot);
        }
        shooterMotor.setPower(power);
        shooterMotor.update();
    }

}
