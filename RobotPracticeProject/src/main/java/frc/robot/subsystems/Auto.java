package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.jni.CANSWDLJNI;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Auto extends SubsystemBase {
  PigeonIMU autoPigeon;

    public Auto(){
      autoPigeon = new PigeonIMU(Gyro.talon);
    }

    public void AutoINIT(){
      driveTrain.leftEnc.reset();
      driveTrain.rightEnc.reset();
      autoPigeon.setFusedHeading(0);
    }

    public void drive(){
        SmartDashboard.putString("Auto Status", "Gyro");
        double kp = 0.05;
        double GyroReading = autoPigeon.getFusedHeading();
        if (GyroReading == 120 || GyroReading == -120){
          GyroReading = 0;
          Robot.pigeon.setFusedHeading(0);
        }
        if(-driveTrain.leftEnc.getDistance() < 10000 && -driveTrain.rightEnc.getDistance() < 10000){
          if (GyroReading == 120 || GyroReading == -120){
            GyroReading = 0;
            Robot.pigeon.setFusedHeading(0);
          }
          GyroReading = autoPigeon.getFusedHeading();
          SmartDashboard.putString("Auto Status", "Driving");  

          double leftSpeed = 0.3 + (kp * GyroReading);
          double rightSpeed = 0.3 - (kp * GyroReading);
          driveTrain.robotDrive.tankDrive(-leftSpeed, rightSpeed);
          
          SmartDashboard.putNumber("Left Auto Speed", leftSpeed);
          SmartDashboard.putNumber("Right Auto Speed", rightSpeed);
          SmartDashboard.putNumber("Auto Gyro", GyroReading);
          SmartDashboard.putNumber("Left Encoder", -driveTrain.leftEnc.getDistance());
          SmartDashboard.putNumber("Right Encoder", -driveTrain.leftEnc.getDistance());
        }
    }

    public void driveWcoordinates(){
      SmartDashboard.putString("Auto Status", "Coordiates");
      double kp = 0.1;
      double GyroReading = autoPigeon.getFusedHeading();
      
      if(driveTrain.totalX <= 40){
        SmartDashboard.putString("Auto Status", "Driving");  

        double leftSpeed = 0.2 - (kp * driveTrain.totalY);
        double rightSpeed = 0.2 + (kp * driveTrain.totalY);
        driveTrain.robotDrive.tankDrive(-leftSpeed, rightSpeed);
        
        SmartDashboard.putNumber("Left Auto Speed", leftSpeed);
        SmartDashboard.putNumber("Right Auto Speed", rightSpeed);
        SmartDashboard.putNumber("Robot y", driveTrain.y);
        SmartDashboard.putNumber("Left Encoder", -driveTrain.leftEnc.getDistance());
        SmartDashboard.putNumber("Right Encoder", -driveTrain.leftEnc.getDistance());
      }
      else{
        SmartDashboard.putString("Auto Status", "Ended");
        driveTrain.robotDrive.stopMotor();
      }
    }

    public void end(){
      driveTrain.robotDrive.stopMotor();
      SmartDashboard.putString("Auto Status", "Ended");
    }
}
