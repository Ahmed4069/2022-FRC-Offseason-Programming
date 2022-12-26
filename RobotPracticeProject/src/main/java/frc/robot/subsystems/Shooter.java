package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
    private TalonFX left, right;
    private TalonSRX hood;
    
    public Shooter(){
        right = new TalonFX(Constants.LEFT_SHOOTER_ID);
        left = new TalonFX(Constants.RIGHT_SHOOTER_ID);
        hood = new TalonSRX(Constants.TALON_SRX_ID_1);
        left.setInverted(true);
        left.follow(right);
    }

    public void shoot(){
        if(Robot.con1.getYButton()){
            right.set(ControlMode.PercentOutput, -1);
        }
        else if(Robot.con1.getXButton()){
            right.set(ControlMode.PercentOutput, -0.75);
        }
        else if(Robot.con1.getBButton()){
            right.set(ControlMode.PercentOutput, -0.5);
        }
        else if(Robot.con1.getAButton()){
            right.set(ControlMode.PercentOutput, -0.25);
        }
        else{
            right.set(ControlMode.PercentOutput,0);
        }
        hood.set(ControlMode.PercentOutput, Robot.con1.getRightY()); 
    }
}
