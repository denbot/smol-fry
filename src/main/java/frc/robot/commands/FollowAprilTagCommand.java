package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.util.LimelightHelpers.RawFiducial;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.Drive;

public class FollowAprilTagCommand extends Command {
    private final boolean enableDrivetrain = false;
    private final double maxSpeed = Meters.of(0.5).in(Meters);
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final double searchAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond);

    private final String limelightName = "limelight";
    
    private final Transform2d targetOffset = new Transform2d(new Translation2d(1.0, 0.0), new Rotation2d(0)); // Desired offset from the tag (0.5 meters in front)

    private final double rotationPd = 2.0; // Proportional gain for rotation control
    private final double translationPd = 1.0; // Proportional gain for translation control

    private Drive drive;

    private int countSinceLastSeen = 0;
    private Pose2d targetPoseFieldSpace = new Pose2d();

    public FollowAprilTagCommand(Drive drive) {
        addRequirements(drive);
        this.drive = drive;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private Transform2d pose2dToTransform2d(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        LimelightResults results = LimelightHelpers.getLatestResults(limelightName);

        LimelightTarget_Fiducial targetFiducial = null;
        for(LimelightTarget_Fiducial fiducial : results.targets_Fiducials) {
            if(fiducial.fiducialID == 1) { // Replace 1 with the desired fiducial ID
                targetFiducial = fiducial;
                break;
            }
        }

        //Get the robot's current pose in the "field's" coordinate space. Note that we don't actually have
        //a true field-relative pose since we don't have a global reference. The 0,0 of the field space is just
        //wherever the robot starts. But we are only using the odometry for short-term pose estimation when
        //we lose sight of the tag, so it doen't really matter.
        Pose2d robotPoseFieldSpace = drive.getPose();
        Logger.recordOutput("FollowAprilTag/RobotPose_FieldSpace", robotPoseFieldSpace);

        if (targetFiducial != null) {
            //Get the tag pose in the robot's coordinate space and then offset
            //it to get the desired target location in the robot's coordinate space.
            //Adding a Transform2d to a Pose2d applies it in the Pose2d's coordinate space.
            Pose2d tagPoseRobotSpace = targetFiducial.getTargetPose_RobotSpace2D();
            Pose2d targetPoseRobotSpace = tagPoseRobotSpace.plus(targetOffset);

            //Get the target pose in the field's coordinate space by adding the target pose in robot space to the robot pose in field space.
            targetPoseFieldSpace = robotPoseFieldSpace.plus(pose2dToTransform2d(targetPoseRobotSpace));

            countSinceLastSeen = 0;

            Logger.recordOutput("FollowAprilTag/TagPose_RobotSpace", tagPoseRobotSpace);
            Logger.recordOutput("FollowAprilTag/TargetPose_RobotSpace", targetPoseRobotSpace);
            Logger.recordOutput("FollowAprilTag/TargetPose_FieldSpace", targetPoseFieldSpace);

        } else {
            countSinceLastSeen++;
        }

        if (countSinceLastSeen < 3) { // If we recently saw the tag, keep trying to go to the last known position
            //Get the XY and rotation error between where the robot is and where we want it to be.
            double adjustedTargetX = targetPoseFieldSpace.getX() - robotPoseFieldSpace.getX();
            double adjustedTargetY = targetPoseFieldSpace.getY() - robotPoseFieldSpace.getY();
            double targetRotation = Math.atan2(adjustedTargetY, adjustedTargetX);

            //Calculate the velocity requests based on that error.
            double xVelocityRequest = clamp(adjustedTargetX * translationPd, -maxSpeed, maxSpeed);
            double yVelocityRequest = clamp(adjustedTargetY * translationPd, -maxSpeed, maxSpeed);
            double rotationRequest = clamp(targetRotation * rotationPd, -maxAngularRate, maxAngularRate);
            
            //Only command the drivetrain if the boolean is set to enable it
            //This just allows easy disabiling of the drivetrain for safer testing.
            if(enableDrivetrain) {
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        xVelocityRequest,
                        yVelocityRequest,
                        rotationRequest);
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        drive.getRotation()));
            } else {
                drive.stopWithX();
            }

            Logger.recordOutput("FollowAprilTag/AdjustedTargetX", adjustedTargetX, Meters);
            Logger.recordOutput("FollowAprilTag/AdjustedTargetY", adjustedTargetY, Meters);
            Logger.recordOutput("FollowAprilTag/TargetRotation", targetRotation, Radians);
            Logger.recordOutput("FollowAprilTag/XVelocityRequest", xVelocityRequest, MetersPerSecond);
            Logger.recordOutput("FollowAprilTag/YVelocityRequest", yVelocityRequest, MetersPerSecond);
            Logger.recordOutput("FollowAprilTag/RotationRequest", rotationRequest, RadiansPerSecond);

            countSinceLastSeen++;
        } else {

            double xVelocityRequest = 0;
            double yVelocityRequest = 0;
            double rotationRequest = searchAngularRate;

            //Only command the drivetrain if the boolean is set to enable it
            //This just allows easy disabiling of the drivetrain for safer testing.
            if(enableDrivetrain){
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        xVelocityRequest,
                        yVelocityRequest,
                        rotationRequest);
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        drive.getRotation()));

            } else {
                drive.stopWithX();
            }

            Logger.recordOutput("FollowAprilTag/XVelocityRequest", xVelocityRequest, MetersPerSecond);
            Logger.recordOutput("FollowAprilTag/YVelocityRequest", yVelocityRequest, MetersPerSecond);
            Logger.recordOutput("FollowAprilTag/RotationRequest", rotationRequest, RadiansPerSecond);
        }

        Logger.recordOutput("FollowAprilTag/CountSinceLastSeen", countSinceLastSeen);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
