package com.gemsrobotics.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Claw extends SubsystemBase{
    public TalonFX pincher;
    public TalonFX wrist;
    public Claw(int pincherModuleNum, int wristModuleNum){
        pincher = new TalonFX(pincherModuleNum);
        wrist = new TalonFX(wristModuleNum);
    }
    public void pinch(){
        //pinch cone
        boolean isOpen = true;
        if (isOpen)
            pincher.set(ControlMode.Velocity, 20, DemandType.ArbitraryFeedForward, 2);
        else
            pincher.set(ControlMode.Velocity, 20, DemandType.ArbitraryFeedForward, 2);
    }
    public void rotateUp(){
        //rotate wrist left
        wrist.set(ControlMode.Velocity, 20, DemandType.ArbitraryFeedForward, 2);
    }
    public void rotateDown(){
        //rotate wrist right
        wrist.set(ControlMode.Velocity, 20, DemandType.ArbitraryFeedForward, 2);
    }
}