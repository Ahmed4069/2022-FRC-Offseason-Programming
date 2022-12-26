package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class intake extends SubsystemBase {

    CANSparkMax drive, articulate;
   
    public intake(){
        drive = new CANSparkMax(Constants.DRIVE_ID, MotorType.kBrushless);
        articulate = new CANSparkMax(Constants.ARTICULATE_ID, MotorType.kBrushless); 
    }

    public void driveIntake(double speed){
        drive.set(-speed);
        
    }

    public void articulateIntake(double speed){
        articulate.set(-speed);
        
    }

    @Override
    public void periodic(){

    }

    @Override
    public void simulationPeriodic(){

    }
}
