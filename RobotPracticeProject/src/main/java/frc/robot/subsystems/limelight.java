package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class limelight extends SubsystemBase{
   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
   NetworkTableEntry tx = table.getEntry("tx"), ty = table.getEntry("ty"), ta = table.getEntry("ta");;
   public static double limelightX, limelightY; 
    double limelightHightInches, hightOfGoal;

    

    public limelight(){
       
    }
    
    public void getDistacetoTarget(){
        limelightX = tx.getDouble(0.0);
        limelightY = ty.getDouble(0.0);
        
        SmartDashboard.putNumber("Distance to Target x", limelightX);
        SmartDashboard.putNumber("Distance to target y", -limelightY);
        
    }
}
