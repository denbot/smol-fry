package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveToPoint extends Command {
  /** Creates a new MoveToPoint. */

  private final CommandSwerveDrivetrain driveSubsystem;
  private final SwerveRequest.RobotCentric drive;
  private final Timer timer = new Timer();

  boolean end = false;
  double kP = 1.3;
  double rotationalKP = -0.05;
  double rotationMult = 1;

  int framesDropped = 0;

  public MoveToPoint(CommandSwerveDrivetrain driveSubsystem, SwerveRequest.RobotCentric drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.drive = drive;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end = false;
    timer.start();
    rotationMult = 1;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] tagPoseRobot = LimelightHelpers.getTargetPose_RobotSpace("");
    Pose3d pose = new Pose3d(new Translation3d(tagPoseRobot[0], tagPoseRobot[1], tagPoseRobot[2]), new Rotation3d(Math.toRadians(tagPoseRobot[3]), Math.toRadians(tagPoseRobot[4]), Math.toRadians(tagPoseRobot[5])));

    if (LimelightHelpers.getTV("")) {
      framesDropped = 0;
    } else {
      framesDropped++;
      if (framesDropped > 5) {
        end = true;
      }
      return;
    }

    Translation3d translate = new Translation3d(0, 0, -2);
    translate = translate.rotateBy(pose.getRotation());
    translate = translate.plus(pose.getTranslation());
    SmartDashboard.putNumber("x", tagPoseRobot[0]);
    SmartDashboard.putNumber("y", tagPoseRobot[1]);
    SmartDashboard.putNumber("z", tagPoseRobot[2]);
    SmartDashboard.putNumber("rx", tagPoseRobot[3]); // pitch
    SmartDashboard.putNumber("ry", tagPoseRobot[4]); // yaw
    SmartDashboard.putNumber("rz", tagPoseRobot[5]); // roll
    SmartDashboard.putNumber("translated x", translate.getX());
    SmartDashboard.putNumber("translated y", translate.getY());
    SmartDashboard.putNumber("translated z", translate.getZ());
    
    double maxVelocity = 3;
    double xDriveSpeed = Math.max(-maxVelocity, Math.min(maxVelocity, kP * translate.getZ()));
    double yDriveSpeed = Math.max(-maxVelocity, Math.min(maxVelocity, kP * translate.getX()));
    SmartDashboard.putNumber("xDriveSpeed", xDriveSpeed);
    SmartDashboard.putNumber("yDriveSpeed", yDriveSpeed);
    
    driveSubsystem.setControl(drive.withVelocityX(xDriveSpeed).withVelocityY(-yDriveSpeed).withRotationalRate(LimelightHelpers.getTX("") * rotationalKP * rotationMult));

    if (LimelightHelpers.getTX("") < 5 && Math.sqrt(Math.pow(translate.getZ(), 2) + Math.pow(translate.getX(), 2)) < 0.25){
      end = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // driveSubsystem.setControl(drive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
