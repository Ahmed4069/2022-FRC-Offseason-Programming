package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class colourWheel extends SubsystemBase {
    CANSparkMax motor;
    public colourWheel(){
        motor = new CANSparkMax(Constants.COLOUR_WHEEL_ID, MotorType.kBrushless);

    }

    public void spin(){
        if(Robot.con2.getYButton()){
            motor.set(-1);
        }
        else if(Robot.con2.getXButton()){
            motor.set(-0.75);
        }
        else if(Robot.con2.getBButton()){
            motor.set(-0.5);
        }
        else if(Robot.con2.getAButton()){
            motor.set(-0.25);
        }
        else{
            motor.set(0);
        } 
    }
}
