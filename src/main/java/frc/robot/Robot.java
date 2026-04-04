// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  Timer m_gcTimer = new Timer();

  public Robot() {
    SignalLogger.enableAutoLogging(false);
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    /**
     * Log some details about the build to the smart dashboard.
     */
    SmartDashboard.putString("Build/ProjectName", BuildConstants.MAVEN_NAME);
    SmartDashboard.putString("Build/BuildDate", BuildConstants.BUILD_DATE);
    SmartDashboard.putString("Build/GitSHA", BuildConstants.GIT_SHA);
    SmartDashboard.putString("Build/GitDate", BuildConstants.GIT_DATE);
    SmartDashboard.putString("Build/GitBranch", BuildConstants.GIT_BRANCH);
  }

  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to non-RT thread priority (do not modify the first argument)
    Threads.setCurrentThreadPriority(false, 10);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    if (m_autonomousCommand != m_robotContainer.getAutonomousCommand()) {
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    }
    // run the garbage collector every 5 seconds
    if (m_gcTimer.advanceIfElapsed(5)) {
      System.gc();
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
    // run the garbage collector every 5 seconds
    if (m_gcTimer.advanceIfElapsed(5)) {
      System.gc();
    }
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // run the garbage collector every 5 seconds
    if (m_gcTimer.advanceIfElapsed(5)) {
      System.gc();
    }
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    // run the garbage collector every 5 seconds
    if (m_gcTimer.advanceIfElapsed(5)) {
      System.gc();
    }
  }

  @Override
  public void testExit() {
  }
}