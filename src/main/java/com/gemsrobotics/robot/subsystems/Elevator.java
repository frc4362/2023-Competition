package com.gemsrobotics.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.math.Conversions;
import com.gemsrobotics.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    
    private TalonFX mStage;
    private TalonFX mPivot;

    private final double upperBoundStage = 0.0;
    private final double lowerBoundStage = 0.0;
    private final double upperBoundPivot = 0.0;
    private final double lowerBoundPivot = 0.0;

    /*elevator */
    public Elevator(int id1, int id2){
       
        mStage = new TalonFX(id1);

        mPivot = new TalonFX(id2);
    }
    //sets to high position @todo
    public void setHigh(){
        
        mStage.set(ControlMode.Position, 1.0);
        mPivot.set(ControlMode.Position, 1.0);
    }
    //sets to mid position @todo
    public void setMid(){
        mStage.set(ControlMode.Position, 1.0);
        mPivot.set(ControlMode.Position, 1.0);
    }
    //sets to hybrid position @todo
    public void setHybrid(){
        mStage.set(ControlMode.Position, 1.0);
        mPivot.set(ControlMode.Position, 1.0);
    }
    //resets stage and pivot to original location @todo
    public void reset(){
        mStage.set(ControlMode.Position, 0.0);
        mPivot.set(ControlMode.Position, 0.0);
    }
}
