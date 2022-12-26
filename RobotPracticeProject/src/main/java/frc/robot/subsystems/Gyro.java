package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gyro extends SubsystemBase {
    public static TalonSRX talon;
    public static PigeonIMU gyro;
    private driveTrain DriveTrain;
    private double gyroReading;

    public Gyro(){
        talon = new TalonSRX(Constants.TALON_SRX_ID_2);
        gyro = new PigeonIMU(talon);
    }
     
    /*public void AutoInitialization(){
        gyroReading = gyro.getYaw();
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
        DriveTrain.DTAutoDrive(gyroReading);
    }*/

    public void UpdateGyroYaw(double speed, double rotation){
        double yaw = gyro.getYaw();
        DriveTrain.DT_Drive(speed, rotation, yaw);
    }

    public void AutoYaw(double speed, double rotation){
        double yaw = gyro.getFusedHeading();
    }
    
    public void periodic(){

    }

    public void simulationPeriodic(){

    }
}
