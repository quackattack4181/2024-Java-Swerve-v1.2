// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.wpilibj.DutyCycle;
// import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public double startTime;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  public static double off = 0.00;

  // Head Raise and Lower Speed
  public static double headSpeed = 0.30;
  public double headSpeedFast = 0.60; // Right stick raise speed

  // Head Shooting Positions
  public static double baseHeadAngle = 107.00; 
  
  public static double topHeadAngle = baseHeadAngle - 90;
  public static double longShootAngle = baseHeadAngle - 41;
  public static double lineShootAngle = baseHeadAngle - 35;
  public static double closeShootAngle = baseHeadAngle - 25;
  
  // Move shooting wheels.
  public static double shootingMotorSpeed = 0.90;
  public double shootingMotorSpeedSlow = 0.10;
  
  // Climbing motors speed.
  private static final double climbSpeed = 0.80;
  
  // Single CIM motor for intake wheels.
  public static TalonSRX Motor5 = new TalonSRX(35);
  public static double intakeMotorSpeed = 0.80;
  
  
  public static CANSparkMax shooterMotor31 = new CANSparkMax(31, MotorType.kBrushless);
  public static CANSparkMax shooterMotor32 = new CANSparkMax(32, MotorType.kBrushless);
  public static CANSparkMax headMotor33 = new CANSparkMax(33, MotorType.kBrushless);
  public static CANSparkMax headMotor34 = new CANSparkMax(34, MotorType.kBrushless);
  public CANSparkMax armMotor36 = new CANSparkMax(36, MotorType.kBrushless);
  public CANSparkMax armMotor37 = new CANSparkMax(37, MotorType.kBrushless);
  
  public boolean headDownLimitReached;
  public boolean headUpLimitReached;
  
  // Head Angle Encoder
  public static DutyCycleEncoder headAngleEncoder = new DutyCycleEncoder(0);
  
  private final XboxController driver = RobotContainer.driver;
  private final XboxController driverTwo = RobotContainer.driverTwo;


  //==========================================
  // Custom Functions <-----------------------
  //==========================================
  public void HeadUp() {
    headMotor33.set(-headSpeed);
    headMotor34.set(-headSpeed);
  };
  public void HeadUpFast() {
    headMotor33.set(-headSpeedFast);
    headMotor34.set(-headSpeedFast);
  };
  public void HeadDown() {
    headMotor33.set(headSpeed);
    headMotor34.set(headSpeed);
  };
  public void HeadDownSlow() {
    headMotor33.set(0.10);
    headMotor34.set(0.10);
  };
  public void HeadUpSlow() {
    headMotor33.set(-0.20);
    headMotor34.set(-0.20);
  };
  public void stopHead() {
    headMotor33.set(off);
    headMotor34.set(off);
  };
  static double getHeadAngle() {
    return headAngleEncoder.getAbsolutePosition()*360;
  }

  // Auto Commands Below
  // SHOOTING & HEAD ANGLE FUNCTIONS <<<<-----------------------------------
  // ========================================================================
  public static void shootRingAuto(double headAngle) {
    double startTime = Timer.getFPGATimestamp(); 
    double liveTime = 0.00;
    while (liveTime < 2.5) {
      shooterMotor31.set(shootingMotorSpeed);
      shooterMotor32.set(shootingMotorSpeed);
      if (liveTime < 1.5) {
        setHeadAngle(headAngle);
      }
      if (liveTime > 1.0) {
        Motor5.set(ControlMode.PercentOutput, -intakeMotorSpeed);
      }
      if (liveTime > 1.5) {
        setHeadAngle(baseHeadAngle);
      }
      liveTime = Timer.getFPGATimestamp() - startTime;
    }
    shooterMotor31.set(off);
    shooterMotor32.set(off);
    Motor5.set(ControlMode.PercentOutput, off);
    headMotor33.set(-off);
    headMotor34.set(-off);
  }

  // ========================================================================
  // Shoots ring close
  public static Command runShootClose() { // <------ runShootRingClose
    return new InstantCommand(() -> {
      shootRingAuto(closeShootAngle);
    });
  }
  // Shoots ring from the line
  public static Command runShootLine() { // <------ runShootRingLine
    return new InstantCommand(() -> {
      shootRingAuto(lineShootAngle);
    });
  }
  // Shoots ring close
  public static Command runShootLong() { // <------ runShootRingLong
    return new InstantCommand(() -> {
      shootRingAuto(longShootAngle);
    });
  }

  // ========================================================================
  public static void Intake1() {
    double startTime = Timer.getFPGATimestamp(); 
    double runTime = 1.00;
    while (Timer.getFPGATimestamp() - startTime < runTime) {
      Motor5.set(ControlMode.PercentOutput, -intakeMotorSpeed);
    }
    Motor5.set(ControlMode.PercentOutput, off);
  }
  // Run intake for 1 second.
  public static Command runIntake1() { // <------------- runIntake1 (1 seconds)
    return new InstantCommand(() -> {
      Intake1();
    });
  }

  // ========================================================================
  public static void Intake2() {
    double startTime = Timer.getFPGATimestamp(); 
    double runTime = 2.00;
    while (Timer.getFPGATimestamp() - startTime < runTime) {
      Motor5.set(ControlMode.PercentOutput, -intakeMotorSpeed);
    }
    Motor5.set(ControlMode.PercentOutput, off);
  }
  // Run intake for 2 second.
  public static Command runIntake2() { // <------------- runIntake2 (2 seconds)
    return new InstantCommand(() -> {
      Intake2();
    });
  }
  // ========================================================================
  public static void Intake3() {
    double startTime = Timer.getFPGATimestamp(); 
    double runTime = 3.00;
    while (Timer.getFPGATimestamp() - startTime < runTime) {
      Motor5.set(ControlMode.PercentOutput, -intakeMotorSpeed);
    }
    Motor5.set(ControlMode.PercentOutput, off);
  }
  // Run intake for 3 second.
  public static Command runIntake3() { // <------------- runIntake3 (3 seconds)
    return new InstantCommand(() -> {
      Intake3();
    });
  }
  // ========================================================================
  public static void IntakeAdjust() {
    double startTime = Timer.getFPGATimestamp(); 
    double runTime = 0.25;
    while (Timer.getFPGATimestamp() - startTime < runTime) {
      Motor5.set(ControlMode.PercentOutput, intakeMotorSpeed*0.40);
    }
    Motor5.set(ControlMode.PercentOutput, off);
  }
  // Adjust's the ring after intake.
  public static Command runIntakeAdjust() { // <-------- runIntakeAdjust (0.25 seconds)
    return new InstantCommand(() -> {
      IntakeAdjust();
    });
  }

  // ========================================================================

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Mod0 - 7 & 8", Swerve.mSwerveMods[0].getState().angle.getDegrees());
    SmartDashboard.putNumber("Mod1 - 1 & 2", Swerve.mSwerveMods[1].getState().angle.getDegrees());
    SmartDashboard.putNumber("Mod2 - 5 & 6", Swerve.mSwerveMods[2].getState().angle.getDegrees());
    SmartDashboard.putNumber("Mod3 - 3 & 4", Swerve.mSwerveMods[3].getState().angle.getDegrees());
    SmartDashboard.putNumber("Head Angle Position", getHeadAngle());
    

    if (driver.getBackButtonPressed()) {
      Swerve.mSwerveMods[0].resetToAbsolute();
      Swerve.mSwerveMods[1].resetToAbsolute();
      Swerve.mSwerveMods[2].resetToAbsolute();
      Swerve.mSwerveMods[3].resetToAbsolute();
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}
  
  
  @Override
  public void disabledPeriodic() {}


  
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Lift The Head Motors
    headMotor33.setInverted(true);
    headMotor34.setInverted(false);
  }

  final static double slowSpeed = 0.30;
  final static double stallSpeed = 0.03;
  public static void setHeadAngle(double targetAngle) {
    if (getHeadAngle() > targetAngle + 5) {
      headMotor33.set(-slowSpeed);
      headMotor34.set(-slowSpeed);
    }
    else if (getHeadAngle() < targetAngle) {
      headMotor33.set(slowSpeed);
      headMotor34.set(slowSpeed);
    } else {
      headMotor33.set(-stallSpeed);
      headMotor34.set(-stallSpeed);
    }
  }

  // ###############################################
  // ###############################################
  public static void shoot() {
    double startTime = Timer.getFPGATimestamp(); 
    double liveTime = 0.00;
    while (liveTime < 1.5) {
      shooterMotor31.set(shootingMotorSpeed);
      shooterMotor32.set(shootingMotorSpeed);
      if (liveTime > 1.0) {
        Motor5.set(ControlMode.PercentOutput, -intakeMotorSpeed);
      }
      liveTime = Timer.getFPGATimestamp() - startTime;
    }
    shooterMotor31.set(off);
    shooterMotor32.set(off);
    Motor5.set(ControlMode.PercentOutput, off);
  }
  // ###############################################
  // ###############################################



  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    /* Driver One Controls *////////////////////////////////////////////////////////////////////////
    
    if (driver.getRightBumper() == true) {
      armMotor37.set(-climbSpeed);
    } else if (driver.getRightTriggerAxis() > 0.50) {
      armMotor37.set(climbSpeed);
    } else {armMotor37.set(off);};

    if (driver.getLeftBumper() == true) {
      armMotor36.set(-climbSpeed);
    } else if (driver.getLeftTriggerAxis() > 0.50) {
      armMotor36.set(climbSpeed);
    } else {armMotor36.set(off);};

    // inverts driving axis's
    if (driver.getStartButtonPressed()) {
      RobotContainer.InvertControlModifier = RobotContainer.InvertControlModifier * -1.00;
    }


    /* Driver Two Controls *////////////////////////////////////////////////////////////////////////

    if (driverTwo.getLeftY() > 0.50) {
      setHeadAngle(baseHeadAngle);
      //Above is the Head all the way down angle.
      
    } else if (driverTwo.getLeftY() < -0.50) {
      setHeadAngle(topHeadAngle);
      //Above is the head all the way up angle.

    } else if (driverTwo.getRightY() < -0.50) {
      HeadUpFast();

    // Head Angle One
    } else if (driverTwo.getXButton()) {
      setHeadAngle(longShootAngle);  //<-------------------------------- Angle #1 (Name Here)

    // Head Angle Two
    } else if (driverTwo.getYButton()) {
      setHeadAngle(closeShootAngle);  //<-------------------------------- Angle #2 (Name Here)

    // Head Angle Three
    } else if (driverTwo.getBButton()) {
      setHeadAngle(lineShootAngle);  //<-------------------------------- Angle #3 (Name Here)

    } else {
      stopHead();
    };

    //************************************************** */
    //************************************************** */
    
    //************************************************** */

    // Shoots the ring
    if (driverTwo.getRightTriggerAxis() > 0.50) {
      shooterMotor31.set(shootingMotorSpeed);
      shooterMotor32.set(shootingMotorSpeed);

    // TEST THIS FOR ONE BUTTON SHOOT
    // } else if (driverTwo.getRightBumperReleased()) {
    //   shoot();

    // Shoots the ring 
    } else if (driverTwo.getLeftTriggerAxis() > 0.50) { 
      shooterMotor31.set(shootingMotorSpeedSlow);
      shooterMotor32.set(shootingMotorSpeedSlow);

    } else {
      shooterMotor31.set(off);
      shooterMotor32.set(off);
    };


    // sucks in the ring
    if (driverTwo.getLeftBumper() == true) {
      Motor5.set(ControlMode.PercentOutput, -intakeMotorSpeed);
    }
    else if (driverTwo.getRightBumper() == true) {
      Motor5.set(ControlMode.PercentOutput, -intakeMotorSpeed);
    }
    // Green wheels push out the ring
    else if (driverTwo.getAButton() == true) {
      Motor5.set(ControlMode.PercentOutput, intakeMotorSpeed*0.50);
    } else {
      Motor5.set(ControlMode.PercentOutput, off);
    };

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
