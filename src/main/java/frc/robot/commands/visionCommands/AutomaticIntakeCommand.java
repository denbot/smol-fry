package frc.robot.commands.visionCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.limelight.LimelightHelpers;
import frc.robot.util.limelight.LimelightHelpers.RawDetection;
import frc.robot.util.limelight.Limelights;

// object detection

// repositioning for intake

// movement for intake

public class AutomaticIntakeCommand extends Command {
  // creates a new AutomaticIntakeCommand
  DoublePublisher publisher;
  DoublePublisher anglePublisher;
  DoublePublisher velocityPublisher;
  double kP = 5;
  double rotationalKP = -0.05;
  int framesDropped = 0;
  double yDirection = 0;
  double xDirection = 0;

  double distance = 0;
  //private final SwerveRequest.RobotCentric 
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withDeadband(1 * 0.1).withRotationalDeadband(1 * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();


  CommandSwerveDrivetrain drive;

  public AutomaticIntakeCommand(CommandSwerveDrivetrain drive) {
    this.drive = drive;
    addRequirements(drive);
    publisher = NetworkTableInstance.getDefault().getDoubleTopic("Distance").publish();
    publisher.set(-3);
    anglePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Angle").publish();
    anglePublisher.set(-3);
    velocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("velocity").publish();
    velocityPublisher.set(-3);
  }

  public void initialize() {
    publisher.set(-2);
    anglePublisher.set(-2);
  }

  public void execute() {
    // if we drop a frame, do nothing for this periodic, unless we've dropped 6 or more frames, in
    // which case we end the command
    if (LimelightHelpers.getTV(Limelights.REAR.name)) {
      framesDropped = 0;
    } else {
      framesDropped++;
      return;
    }

    // gets raw data from the limelight
    RawDetection[] rawDetection = LimelightHelpers.getRawDetections("limelight-rear");

    if (rawDetection.length != 0) {
      yDirection = 90 + rawDetection[0].tync;
      xDirection = rawDetection[0].txnc; // this will be used for repositioning
      distance = Math.tan(Math.toRadians(yDirection)) * 5.5625;
      publisher.set(distance);
      anglePublisher.set(xDirection);
    } else {
      publisher.set(-1);
      anglePublisher.set(0);
    }
    
    drive.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(xDirection*rotationalKP));
    velocityPublisher.set(xDirection*rotationalKP);
    //ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,xDirection*rotationalKP);
//    drive.runVelocity(chassisSpeeds);

    /*
    double maxVelocity = 0.5; // TODO: When in large space set to 6
    double xDriveSpeed = Math.max(-maxVelocity, Math.min(maxVelocity, kP * Math.sin(distance)));
    SmartDashboard.putNumber("xDriveSpeed", xDriveSpeed);
    double yDriveSpeed = Math.max(-maxVelocity, Math.min(maxVelocity, kP * Math.cos(distance)));
    SmartDashboard.putNumber("yDriveSpeed", yDriveSpeed);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xDriveSpeed, yDriveSpeed, angle * rotationalKP);

    drive.runVelocity(chassisSpeeds);*/

  }

  public void end(boolean interrupted) {
    // drive.stopWithX();
    SmartDashboard.putString("done", "done");
  }

  public boolean isFinished() {
    return false;
    // return (Math.abs(angle) < 3 && distance < 0.1) || (framesDropped > 5);
  }
}
