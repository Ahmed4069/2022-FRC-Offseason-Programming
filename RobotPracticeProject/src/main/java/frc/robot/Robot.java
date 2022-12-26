package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.colourWheel;
import frc.robot.subsystems.driveTrain;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.tower;

public class Robot extends TimedRobot {
  
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private intake Intake;
  private Gyro gyro;
  private driveTrain DriveTrain;
  private TalonSRX talon;
  public static PigeonIMU pigeon;
  public static XboxController con1, con2;
  private double startPos;
  private Auto auto;
  private Shooter shooter;
  private tower Tower;
  private limelight LimeLight;
  private Climber climber;
  private colourWheel colour;
  private RobotContainer container;
  

  @Override
  public void robotInit() {
    talon = new TalonSRX(Constants.TALON_SRX_ID_2);
    pigeon = new PigeonIMU(talon);
    Intake = new intake();
    DriveTrain = new driveTrain();
    con1 = new XboxController(Constants.con1Port);
    con2 = new XboxController(1);
    gyro = new Gyro();
    auto = new Auto();
    shooter = new Shooter();
    Tower = new tower();
    
    colour = new colourWheel();
    climber = new Climber();
    container = new RobotContainer(DriveTrain);
    LimeLight = new limelight();
  }
  @Override
  public void robotPeriodic() { 
    if (pigeon.getFusedHeading() == 360){
      pigeon.setFusedHeading(0);    
    }
    LimeLight.getDistacetoTarget();
  }

  @Override
  public void autonomousInit() {
    DriveTrain.resetDriveTrain();
    container.getAutonomousCommand().schedule();
  }

  @Override
  public void autonomousPeriodic() {                         
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    pigeon.setFusedHeading(0);
    auto.end();
    DriveTrain.resetDriveTrain();
  }

   @Override
  public void teleopPeriodic() {
    double speed = con1.getLeftTriggerAxis() - con1.getRightTriggerAxis();
    double rotation = con1.getLeftX();
    double yaw = pigeon.getFusedHeading();
    SmartDashboard.putNumber("Gyro", yaw);
    Intake.driveIntake(con2.getLeftTriggerAxis()-con2.getRightTriggerAxis());
    Intake.articulateIntake(-con2.getLeftY());
    
    DriveTrain.DT_Drive(speed, rotation, yaw);
    DriveTrain.coordinates();
    //LimeLight.getDistacetoTarget();
    shooter.shoot();
    Tower.driveTower();
    colour.spin();
    climber.drive();
   }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
  
  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {
    
  }
}
