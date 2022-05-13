/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private DifferentialDrive m_drive;
  private Joystick m_joy = new Joystick(0);
  private CANSparkMax m_leftAft;
  private CANSparkMax m_leftFront;
  private CANSparkMax m_rightAft;
  private CANSparkMax m_rightFront;

  private Encoder m_leftEncoder = new Encoder(0, 1);
  private Encoder m_rightEncoder = new Encoder(2, 3);
  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  private AnalogGyro m_gyro = new AnalogGyro(1);
  private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftFront, m_leftAft);
  MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightFront, m_rightAft);
  
  DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
  DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
  .85,                    // 7.29:1 gearing reduction.
  7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
  23,                    // The mass of the robot is 60 kg.
  Units.inchesToMeters(6), // The robot uses 3" radius wheels.
  0.6985,                  // The track width is 0.7112 meters.

  // The standard deviations for measurement noise:
  // x and y:          0.001 m
  // heading:          0.001 rad
  // l and r velocity: 0.1   m/s
  // l and r position: 0.005 m
  VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  private Field2d m_field = new Field2d();

  //DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
  //getHeading(), new Pose2d(5.0, 13.5, new Rotation2d()));


  public void Drivetrain() {

    m_leftEncoder.setDistancePerPulse(2 * Math.PI * 6 / 512);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * 6 / 512);
    SmartDashboard.putData("Field", m_field);


  }



  @Override
  public void robotInit() {
  

    m_leftAft = new CANSparkMax(6, MotorType.kBrushless);
    m_leftFront = new CANSparkMax(7, MotorType.kBrushless);
    m_rightAft = new CANSparkMax(9, MotorType.kBrushless);
    m_rightFront = new CANSparkMax(8, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftAft.restoreFactoryDefaults();
    m_leftFront.restoreFactoryDefaults();
    m_rightAft.restoreFactoryDefaults();
    m_rightFront.restoreFactoryDefaults();

    m_rightAft.setInverted(true);
    m_rightFront.setInverted(true);

    

    m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  
  }

  @Override
  public void teleopPeriodic() {

    m_drive.curvatureDrive(-m_joy.getY(), m_joy.getX(), false);

    // This will get the simulated sensor readings that we set
    // in the previous article while in simulation, but will use
    // real values on the robot itself.

    //m_odometry.update(m_gyro.getRotation2d(),
    //m_leftEncoder.getDistance(),
    //m_rightEncoder.getDistance());
    //m_field.setRobotPose(m_odometry.getPoseMeters());
    
  }

  public void simulationPeriodic() {
    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
    m_driveSim.setInputs(m_leftGroup.get() * RobotController.getInputVoltage(),
                         m_rightGroup.get() * RobotController.getInputVoltage());
  
    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_driveSim.update(0.02);
  
    // Update all of our sensors.
    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
  }
}