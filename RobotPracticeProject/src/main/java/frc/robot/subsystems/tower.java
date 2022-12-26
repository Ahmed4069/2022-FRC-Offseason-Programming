package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class tower extends SubsystemBase{

    private CANSparkMax neo;
    
    public tower(){
        neo = new CANSparkMax(Constants.TOWER_ID, MotorType.kBrushless);
        
        
        

    }

    public void driveTower(){
        neo.set(-Robot.con2.getRightY());
    }
        

    public void periodic(){

    }

    public void simulationPeriodic(){

    }
}
