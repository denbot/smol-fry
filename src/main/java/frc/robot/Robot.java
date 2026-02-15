// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import bot.den.visionsim.AprilTag;
import bot.den.visionsim.Camera;
import bot.den.visionsim.VisionSim;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private Field2d robotPosition = new Field2d();

  VisionSim visionSim = new VisionSim();

  public Robot() {
        // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    SmartDashboard.putData(robotPosition);
    m_robotContainer = new RobotContainer();

    if(Robot.isSimulation()) {
      Matrix<N3, N3> intrinsics = new Matrix<N3, N3>(Nat.N3(), Nat.N3());
      intrinsics.set(0, 0, 748);
      intrinsics.set(1, 1, 747);
      intrinsics.set(2, 2, 1);
      intrinsics.set(0, 2, 661);
      intrinsics.set(1, 2, 396);

      visionSim.missedTagDetectionPercent(0.1).
                tagDetectionPositionStdDev(0.025).
                tagDetectionRotationStdDev(Math.toRadians(5));

      visionSim.setDoLimelightPublishing(true);
      visionSim.addCamera(
          new Camera(1280,
                    800,
                    new Pose3d(0.19, -0.04, 0.194, new Rotation3d(Math.toRadians(180), 0, Math.toRadians(180))),
                    intrinsics,
                    "limelight"));

      visionSim.addAprilTag(new AprilTag(0, new Pose3d(2, 0, 0.5, new Rotation3d(0, 0, 0))));
    }

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotPosition.setRobotPose(m_robotContainer.getRobotPosition());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    visionSim.update(m_robotContainer.getRobotPosition());
  }
}
