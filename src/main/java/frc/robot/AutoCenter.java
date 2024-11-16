package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoCenter extends Command {
  
  boolean end = false;
  double kP = -0.05;
  private final Timer timer = new Timer();
  private final CommandSwerveDrivetrain driveSubsystem;
  private final SwerveRequest.FieldCentric drive;
  
  /** Creates a new AutoCenter. */
  public AutoCenter(CommandSwerveDrivetrain driveSubsystem, SwerveRequest.FieldCentric drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    end = false;
    if (LimelightHelpers.getTX("") == 0) {
        end = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setControl(drive.withRotationalRate(LimelightHelpers.getTX("")*kP));
    if (Math.abs(LimelightHelpers.getTX("")) < 5) {
      timer.start();
    } else {
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (end || timer.get() > 0.5);
  }
}
