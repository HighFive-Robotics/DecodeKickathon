package org.firstinspires.ftc.teamcode.Recode.Testing;



import static org.firstinspires.ftc.teamcode.Recode.Module.Outtake.Pusher.retractedPose;
import static org.firstinspires.ftc.teamcode.Recode.Module.Outtake.Trap.closedPose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Recode.Module.Outtake.Mixer;
import org.firstinspires.ftc.teamcode.Recode.Module.Outtake.Pusher;
import org.firstinspires.ftc.teamcode.Recode.Module.Outtake.Shooter;
import org.firstinspires.ftc.teamcode.Recode.Module.Outtake.Trap;

import java.util.HashMap;
import java.util.Timer;

@TeleOp
public class Subsystems extends LinearOpMode {
    private enum System {
        Mixer,
        Shooter,
        Pusher,
        Trap,
    }
    private System currentSystem;
    public Shooter shooter;
    public Mixer mixer;
    public Pusher pusher;
    public Trap trap;
    HashMap<Integer , System> modes = new HashMap<>();
    HashMap<String, ElapsedTime> timers = new HashMap<>();
    int i;
    public void handleInit(){
        shooter = new Shooter(hardwareMap);
        mixer = new Mixer(hardwareMap,0,false);
        pusher = new Pusher(hardwareMap,retractedPose,false);
        trap = new Trap(hardwareMap ,closedPose , false);
        // ---
        // SUBSISTEME
        // ---
        modes.put(1,System.Mixer);
        modes.put(2,System.Shooter);
        modes.put(3,System.Pusher);
        modes.put(4,System.Trap);
        i=1;currentSystem = modes.get(i);
        // ---
        // TIMERS
        // ---
        timers.put("a" , new ElapsedTime());
        timers.put("b" , new ElapsedTime());
        timers.put("x" , new ElapsedTime());
        timers.put("y" , new ElapsedTime());
        timers.put("left", new ElapsedTime());
        timers.put("right" , new ElapsedTime());

        timers.get("left").reset();
        timers.get("right").reset();
        timers.get("a").reset();
        timers.get("b").reset();
        timers.get("x").reset();
        timers.get("y").reset();
        // ---
        // TELEMETRIE
        // ---
        telemetry.addLine("Lista de subsisteme posibile \n 1)Mixer\n 2)Shooter \n 3)Pusher\n 4)Trap");
        telemetry.update();
    }
    public void handleTelemetry() {
        telemetry.addData("TESTING SUBSYSTEM", ">> " + currentSystem.toString() + " <<");
        telemetry.addLine("------------------------------------");
        telemetry.addLine("Use DPAD Left/Right to switch subsystem.");
        telemetry.addLine();
        telemetry.addLine("---CONTROLS---");

        switch (currentSystem) {
            case Mixer:
                telemetry.addLine("A: Go to Slot 1");
                telemetry.addLine("B: Go to Slot 2");
                telemetry.addLine("X: Go to Slot 3");
                break;
            case Shooter:
                telemetry.addLine("A: Start Shooter");
                telemetry.addLine("B: Stop Shooter");
                break;
            case Pusher:
                telemetry.addLine("A: Extend Pusher");
                telemetry.addLine("B: Retract Pusher");
                break;
            case Trap:
                telemetry.addLine("A: Open Trap");
                telemetry.addLine("B: Close Trap");
                break;
        }

        telemetry.addLine();
        telemetry.addLine("---DEBUG INFO---");

        switch (currentSystem) {
            case Mixer:
                telemetry.addData("Current State", mixer.getState());
                telemetry.addData("Servo Target", mixer.getTarget());
                break;
            case Shooter:
                telemetry.addData("Current State", shooter.getState());
                telemetry.addData("Is Ready to Shoot?", shooter.readyToShoot());
                break;
            case Pusher:
                telemetry.addData("Current State", pusher.state);
                telemetry.addData("Servo Target", pusher.getTarget());
                break;
            case Trap:
                telemetry.addData("Curent State" , trap.state);
                telemetry.addData("Servo Target" , trap.getTarget());
                break;
        }
        telemetry.update();
    }
    public void handleTesting(){
            switch (currentSystem){
                case Mixer:
                    if(gamepad1.a && timers.get("a").milliseconds() > 250){
                        mixer.setState(Mixer.States.Slot1);
                        timers.get("a").reset();
                    }
                    if(gamepad1.b && timers.get("b").milliseconds() > 250){
                        mixer.setState(Mixer.States.Slot2);
                        timers.get("b").reset();
                    }
                    if (gamepad1.x && timers.get("x").milliseconds() > 250){
                        mixer.setState(Mixer.States.Slot3);
                        timers.get("x").reset();
                    }
                    break;
                case Shooter:
                    if (gamepad1.a && timers.get("a").milliseconds() > 250){
                        shooter.shoot();
                        timers.get("a").reset();
                    }
                    if(gamepad1.b && timers.get("b").milliseconds() > 250){
                        shooter.stop();
                        timers.get("b").reset();
                    }
                    break;
                case Pusher:
                    if(gamepad1.a && timers.get("a").milliseconds() > 250){
                        pusher.setState(Pusher.States.Extended , 0);
                        timers.get("a").reset();
                    }
                    if(gamepad1.b && timers.get("b").milliseconds() > 250){
                        pusher.setState(Pusher.States.Retracted , 0);
                        timers.get("b").reset();
                    }
                    break;
                case Trap:
                    if(gamepad1.a && timers.get("a").milliseconds() > 250){
                        trap.setState(Trap.States.Open , 0);
                        timers.get("a").reset();
                    }
                    if(gamepad1.b && timers.get("b").milliseconds() > 250){
                        trap.setState(Trap.States.Closed , 0);
                        timers.get("b").reset();
                    }
                    break;
            }
    }
    public void handleUpdates(){
        mixer.update();
        shooter.update();
        pusher.update();
        trap.update();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        handleInit();
        waitForStart();
        telemetry.clearAll();
        while(opModeIsActive()){
            if(gamepad1.dpad_left && timers.get("left").milliseconds() > 250){
                if(i == 1){
                    i = 4;
                    currentSystem = modes.get(i);
                }else currentSystem = modes.get(--i);
                timers.get("left").reset();
            }
            if(gamepad1.dpad_right && timers.get("right").milliseconds() > 250){
                if(i == 4){
                    i = 1;
                    currentSystem = modes.get(i);
                }else currentSystem=modes.get(++i);
                timers.get("right").reset();
            }
            handleTesting();
            handleTelemetry();
            handleUpdates();
        }
    }
}
