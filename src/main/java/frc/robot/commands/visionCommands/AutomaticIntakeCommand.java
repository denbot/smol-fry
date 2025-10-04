package frc.robot.commands.visionCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.limelight.LimelightHelpers;
import frc.robot.util.limelight.LimelightHelpers.RawDetection;
import frc.robot.util.limelight.Limelights;

public class AutomaticIntakeCommand extends Command {
  DoublePublisher distancePublisher;
  DoublePublisher anglePublisher;
  DoublePublisher velocityPublisher;
  StringPublisher statePublisher;

  int framesDropped = 0;
  double yDirection = 0;
  double xDirection = 0;
  double distanceSetpoint = 16;
  double aspectRatio = Double.MAX_VALUE;
  double boundingBoxX = 0;
  double boundingBoxY = 0;
  double distance = 0;

  final double maxYSpeed = 0.25;
  final double maxXSpeed = 0.25;
  final double maxRspeed = 1;
  final double kP = -0.25;
  final double rotationalKP = -0.1;

  enum AutoIntakeState{
    NOT_FOUND,
    FOUND,
    ALIGNED
  };
  
  AutoIntakeState stateOfAutointake = AutoIntakeState.NOT_FOUND;

  private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
      .withDeadband(1 * 0.1).withRotationalDeadband(1 * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  CommandSwerveDrivetrain drive;

  public AutomaticIntakeCommand(CommandSwerveDrivetrain drive) {
    this.drive = drive;
    addRequirements(drive);
    distancePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Distance").publish();
    distancePublisher.set(0);
    anglePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Angle").publish();
    anglePublisher.set(0);
    velocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("velocity").publish();
    velocityPublisher.set(0);

    statePublisher = NetworkTableInstance.getDefault().getStringTopic("AutoIntakeState").publish();
    statePublisher.set("COMMAND CREATED");
  }

  public void initialize() {
    framesDropped = 0;
    stateOfAutointake = AutoIntakeState.NOT_FOUND;
    statePublisher.set(stateOfAutointake.toString());
  }

  public void execute() {
    // gets raw data from the limelight
    RawDetection[] rawDetection = LimelightHelpers.getRawDetections("limelight-rear");
    int targetCorral = -1;
    double distanceSmallest = Double.MAX_VALUE;
    for (int i = 0; i < rawDetection.length; i++){
        if (rawDetection[i].classId == 1) {
          double boundingBox2X = (rawDetection[i].corner0_X + rawDetection[i].corner2_X) / 2;
          double boundingBox2Y = (rawDetection[i].corner0_Y + rawDetection[i].corner2_Y) / 2;
          double distanceCurrent = Math.sqrt(Math.pow(boundingBox2X - boundingBoxX,2) + 
                                            Math.pow(boundingBox2Y - boundingBoxY,2));
          if(distanceCurrent < distanceSmallest) {
            distanceSmallest = distanceCurrent;
            targetCorral = i;
        }
      }
    }

    if (targetCorral != -1 && stateOfAutointake == AutoIntakeState.NOT_FOUND){
      stateOfAutointake = AutoIntakeState.FOUND;
    }
    else if (framesDropped > 5 && stateOfAutointake == AutoIntakeState.FOUND){
      stateOfAutointake = AutoIntakeState.NOT_FOUND;
    }
    else if (aspectRatio < 1.1 && stateOfAutointake == AutoIntakeState.FOUND){
      stateOfAutointake = AutoIntakeState.ALIGNED;
    }
    statePublisher.set(stateOfAutointake.toString());


    if (targetCorral != -1 && stateOfAutointake == AutoIntakeState.FOUND) {
      framesDropped = 0;
      
      yDirection = 90 + rawDetection[targetCorral].tync;
      xDirection = rawDetection[targetCorral].txnc; // this will be used for repositioning
      distance = Math.tan(Math.toRadians(yDirection)) * 5.5625;
      
      distancePublisher.set(distance);
      anglePublisher.set(xDirection);
      
      aspectRatio = Math.abs((rawDetection[targetCorral].corner0_X - rawDetection[targetCorral].corner2_X) /
                             (rawDetection[targetCorral].corner0_Y - rawDetection[targetCorral].corner2_Y));
      
      boundingBoxX = (rawDetection[targetCorral].corner0_X + rawDetection[targetCorral].corner2_X) / 2;
      boundingBoxY = (rawDetection[targetCorral].corner0_Y + rawDetection[targetCorral].corner2_Y) / 2;
      
      double yInput = (distanceSetpoint - distance) * kP;
      if (Math.abs(distanceSetpoint - distance) < 2) {
        yInput = 0;
      }

      if (yInput > maxYSpeed) {
        yInput = maxYSpeed;
      }
      else if (yInput < -maxYSpeed) {
        yInput = -maxYSpeed;
      }

      double xInput = maxXSpeed;
      if (aspectRatio < 1.1) {
        xInput = 0;
      }
     drive.setControl(robotCentricRequest.withVelocityX(xInput)
                                          .withVelocityY(yInput)
                                          .withRotationalRate(xDirection*rotationalKP));

      velocityPublisher.set(yInput);
    }
    else if (stateOfAutointake == AutoIntakeState.NOT_FOUND){
      drive.setControl(robotCentricRequest.withVelocityX(0)
                                            .withVelocityY(0)
                                            .withRotationalRate(maxRspeed));
          
    }

    if (targetCorral != -1){
      anglePublisher.set(framesDropped);
      framesDropped++;
    }
  }
  
  public void end(boolean interrupted) {
    drive.setControl(brake);
    statePublisher.set("END"); 
  }

  public boolean isFinished() {
    return (stateOfAutointake == AutoIntakeState.ALIGNED);
  }
}
