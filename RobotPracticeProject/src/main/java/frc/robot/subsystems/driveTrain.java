package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class driveTrain extends SubsystemBase{

    private CANSparkMax leftMaster, rightMaster, leftSlave, rightSlave;
    public static DifferentialDrive robotDrive;
    private SlewRateLimiter speedFilter, turnFilter, rightAutoFilter;
    public static Encoder leftEnc, rightEnc;
    public static double y,x, totalY, totalX;
    private static DifferentialDriveOdometry m_Odometry;
    private DifferentialDriveKinematics kinematics;
    private Robot robot;
    private Pose2d pose;
    private Rotation2d rotation;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.187, 0.368, 0.227);   

    private double averageEncoderDistance, lastEncoder;

    PIDController leftPidController = new PIDController(0, 0, 0);
    PIDController righPidController = new PIDController(0, 0, 0);

    public driveTrain(){
        leftMaster = new CANSparkMax(Constants.LEFT_MASTER_ID, MotorType.kBrushless);
        leftSlave = new CANSparkMax(Constants.LEFT_SLAVE_ID, MotorType.kBrushless);

        rightMaster = new CANSparkMax(Constants.RIGHT_MASTER_ID, MotorType.kBrushless);
        rightSlave = new CANSparkMax(Constants.RIGHT_SLAVE_ID, MotorType.kBrushless);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftMaster.setInverted(true);

        robotDrive = new DifferentialDrive(leftMaster, rightMaster);

        speedFilter = new SlewRateLimiter(1.8);
        turnFilter = new SlewRateLimiter(1.8);
        rightAutoFilter = new SlewRateLimiter(1.8);

        leftEnc = new Encoder(Constants.LEFT_A_ID, Constants.LEFT_B_ID);
        rightEnc = new Encoder(Constants.RIGHT_A_ID, Constants.RIGHT_B_ID);
        
        leftEnc.reset();
        rightEnc.reset();

        leftEnc.setDistancePerPulse(1.57079632679 / 2048);
        rightEnc.setDistancePerPulse(1.57079632679 / 2048);

        kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(20));

        m_Odometry = new DifferentialDriveOdometry(getAngle());
    }

    public void resetDriveTrain(){
        leftEnc.reset();
        rightEnc.reset();
        Robot.pigeon.setFusedHeading(0);
        lastEncoder = 0;
        totalX = 0;
        totalY = 0;
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        leftMaster.setInverted(true);
       // robotDrive = new DifferentialDrive(leftMaster, rightMaster);
    }

    
    
    

    public void coordinates(){
       double currentEncoder = (-leftEnc.getDistance() + rightEnc.getDistance()) / 2;

        double averageChange = currentEncoder - lastEncoder;

        double angdeg = Robot.pigeon.getFusedHeading();

        double changeRads = Math.toRadians(angdeg);

        y = -Math.sin(changeRads) * averageChange;
        x = -Math.cos(changeRads) * averageChange;

        totalX = totalX + x;
        totalY = totalY + y;

        SmartDashboard.putNumber("x Coordinate", totalX);
        SmartDashboard.putNumber("y Coordinate", totalY);
        SmartDashboard.putNumber("Heading", angdeg);
        
        SmartDashboard.putNumber("RightEnc", rightEnc.getDistance());
        SmartDashboard.putNumber("leftEnc", leftEnc.getDistance());

        lastEncoder = currentEncoder;
    }

    public Rotation2d getAngle(){
        return new Rotation2d(Math.toRadians(Robot.pigeon.getFusedHeading()));
    }

    public PIDController getleftPidController(){
        return leftPidController;
    }

    public PIDController getRighPidController(){
        return righPidController;
    }
    
    public DifferentialDriveKinematics setKinematics(){
        return kinematics;
    }

    public Pose2d getPose(){
        return pose;
    }

    public DifferentialDriveWheelSpeeds getSpeeds(){
        return new DifferentialDriveWheelSpeeds(leftEnc.getRate(), rightEnc.getRate());
    }

    public double getRightSpeed(){
        return rightEnc.getRate();
    }

    public double getLeftSpeed(){
        return leftEnc.getRate();
    }
    public void setOutput (double leftVolts, double rightVolts){
        //robotDrive.tankDrive(leftVolts/12, rightVolts/12);
        leftMaster.set(leftVolts / 12);
        leftSlave.set(leftVolts / 12);
        rightMaster.set(rightVolts / 12);
        rightSlave.set(rightVolts / 12);
        SmartDashboard.putNumber("rightVolts", rightVolts/12);
        SmartDashboard.putNumber("leftVolts", leftVolts/12);
        SmartDashboard.putNumber("right Rate", leftEnc.getRate());
        SmartDashboard.putNumber(" left Rate", rightEnc.getRate());
    }


    @Override
    public void periodic(){
        rotation = Rotation2d.fromDegrees(Robot.pigeon.getYaw());
        pose = m_Odometry.update(getAngle(), getRightSpeed(), getLeftSpeed());

    }

    @Override
    public void simulationPeriodic(){
        
    }

    public SimpleMotorFeedforward getFeedforward(){
        return feedforward;
    }
    public void DT_Drive(double rotation, double speed, double yaw){
        robotDrive.arcadeDrive(speedFilter.calculate(-speed), turnFilter.calculate(-rotation));

        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putNumber("Turn", rotation);
        SmartDashboard.putNumber("Gyro Yaw", yaw);

        if (Robot.con1.getAButton() || Robot.con1.getBButton() || Robot.con1.getXButton() || Robot.con1.getYButton() && limelight.limelightX >= 0.2 || limelight.limelightX <=-0.2){
            double leftSpeed = -(0.01 * limelight.limelightX);
            double rightSpeed = (0.01 * limelight.limelightX);
            //SlewRateLimiter leftLimiter = new SlewRateLimiter(1.3);
            //SlewRateLimiter rightLimiter = new SlewRateLimiter(1.3);
            SmartDashboard.putNumber("Val1", leftSpeed);
            SmartDashboard.putNumber("Val2", rightSpeed);
            robotDrive.tankDrive(leftSpeed, rightSpeed);
        }
       
            
        
    }

    

    
}
