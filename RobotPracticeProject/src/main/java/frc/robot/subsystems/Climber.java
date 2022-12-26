package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Climber extends SubsystemBase{
    TalonSRX talon;
    private CANSparkMax motor, slave;
    public Climber(){
        motor = new CANSparkMax(Constants.RIGHT_CL_ID, MotorType.kBrushless);
        slave = new CANSparkMax(Constants.LEFT_CL_ID, MotorType.kBrushless);
        talon = new TalonSRX(Constants.TALON_SRX_ID_2);
        slave.follow(motor);
    }

    public void drive(){
        /*if (Robot.con1.getRightBumper()){
            motor.set(-0.5);
        }
        else if(Robot.con1.getLeftBumper()){
            motor.set(0.5);
        }
        else{
            motor.set(0);
        }*/


        if(Robot.con2.getLeftBumper()){
            talon.set(ControlMode.PercentOutput, -0.5);
        }
        else if(Robot.con2.getRightBumper()){
            talon.set(ControlMode.PercentOutput, 0.5);
        }
        else{
            talon.set(ControlMode.PercentOutput, 0);
        }
    }
}
