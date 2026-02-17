package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;

public class FollowAprilTagCommand extends Command {
    private final boolean enableDrivetrain = true;
    private double maxSpeed;
    private double maxAngularRate;
    private double searchAngularRate;

    private double rotationDeadband = maxAngularRate * 0.25;
    private double translationDeadband = maxSpeed * 0.25;

    private final String limelightName = "limelight";
    
    //This should really be moved elsewhere
    private final Pigeon2 pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
    private final StatusSignal<LinearAcceleration> accelerationX = pigeon.getAccelerationX();
    private final StatusSignal<LinearAcceleration> accelerationY = pigeon.getAccelerationY();
    private final StatusSignal<LinearAcceleration> accelerationZ = pigeon.getAccelerationZ();

    // Desired offset from the tag. The reference frame for this is X+ right, Y+ down, Z- Out of tag, when looking at the tag.
    //Limelight defines a left-handed frame for tags with Z+ out, but we use right-hand frames so this becomes Z-.
    private final Transform3d targetOffset = new Transform3d(new Translation3d(0, 0.0, -1.5), new Rotation3d(0, 0, 0));
    private ProfiledPIDController  xPositionPID;
    private ProfiledPIDController  yPositionPID;
    private ProfiledPIDController  rotationPID;

    // The desired robot rotation relative to the tag (facing the tag)
    private final Rotation2d targetRobotRotationFacingTag = Rotation2d.fromDegrees(0); 

    // Kalman filter parameters
    private static final double dtSeconds = 0.02; // 20ms loop time
    private static final double modelStdDev = 0.3; // process noise
    private static final double measurementStdDev = 0.15; // Measurement noise from vision
    
    private KalmanFilter<N2, N2, N2> targetPoseFilter;
    private boolean filterInitialized = false;

    private Drive drive;

    private int countSinceLastSeen = 10000; //Start large so we go straight into the searching behavior
    private final int translationStopCount = 20;
    private final int rotationStopCount = 20;

    private Pose3d filteredTargetPoseFieldSpace = new Pose3d();
    private Pose3d tagPoseFieldSpace = new Pose3d();

    public FollowAprilTagCommand(Drive drive) {
        addRequirements(drive);
        this.drive = drive;
        createFilter();

        accelerationX.setUpdateFrequency(50);
        accelerationY.setUpdateFrequency(50);
        accelerationZ.setUpdateFrequency(50);

        //Different tuning is used because the drive system is poorly tuned so the simulation 
        //does not match the real system well.
        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                maxSpeed = Meters.of(2.0).in(Meters);
                maxAngularRate = RotationsPerSecond.of(2.0).in(RadiansPerSecond);
                searchAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond);
                rotationDeadband = maxAngularRate * 0.05;
                translationDeadband = maxSpeed * 0.2;

                xPositionPID = new ProfiledPIDController (5.0, 0, 0.0, 
                                        new TrapezoidProfile.Constraints(maxSpeed, 4));
                yPositionPID = new ProfiledPIDController (5.0, 0, 0.0, 
                                        new TrapezoidProfile.Constraints(maxSpeed, 4));
                rotationPID = new ProfiledPIDController (10.0, 0, 0.0, 
                                        new TrapezoidProfile.Constraints(maxAngularRate, 3));

                break;

            case SIM:
                maxSpeed = Meters.of(0.5).in(Meters);
                maxAngularRate = RotationsPerSecond.of(2.0).in(RadiansPerSecond);
                searchAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond);
                rotationDeadband = maxAngularRate * 0.1;
                translationDeadband = maxSpeed * 0.2;

                xPositionPID = new ProfiledPIDController (1.0, 0, 0, 
                                        new TrapezoidProfile.Constraints(maxSpeed, 3));
                yPositionPID = new ProfiledPIDController (1.0, 0, 0, 
                                        new TrapezoidProfile.Constraints(maxSpeed, 3));
                rotationPID = new ProfiledPIDController (2.0, 0, 1.0, 
                                        new TrapezoidProfile.Constraints(maxAngularRate, 3));
                break;
        }
    }

    private void createFilter() {
        // State: [x, y], Input: [delta_x, delta_y] (none actually used), Output: [x, y]
        // A = 0 (The state does not change based on the previous state since we are not tracking velocity)
        // B = I (identity - for prediction step)
        // C = I (identity - we observe the full state)
        // D = 0 (no feedthrough)
        
        Matrix<N2, N2> A = new Matrix<>(Nat.N2(), Nat.N2());
        Matrix<N2, N2> B = Matrix.eye(Nat.N2());
        Matrix<N2, N2> C = Matrix.eye(Nat.N2());
        Matrix<N2, N2> D = new Matrix<>(Nat.N2(), Nat.N2());
        
        LinearSystem<N2, N2, N2> system = new LinearSystem<>(A, B, C, D);
        
        targetPoseFilter = new KalmanFilter<>(
            Nat.N2(),
            Nat.N2(),
            system,
            VecBuilder.fill(modelStdDev, modelStdDev), // Model state std devs
            VecBuilder.fill(measurementStdDev, measurementStdDev), // Measurement std devs
            dtSeconds
        );        
    }

    private void updateFilter(Pose3d measurement) {
        //Filter the tag pose in field space
        if(!filterInitialized) {
            if(measurement != null) {
                targetPoseFilter.setXhat(VecBuilder.fill(
                    measurement.getX(),
                    measurement.getY()
                    ));
                filterInitialized = true;
            }
        } else {
            // Predict step (Assuming the tag is stationary, so input is zero)
            targetPoseFilter.predict(VecBuilder.fill(0, 0), dtSeconds);
            
            if(measurement != null) {

                double acceleration = Math.sqrt(Math.pow(accelerationX.getValue().in(MetersPerSecondPerSecond), 2) +
                                            Math.pow(accelerationY.getValue().in(MetersPerSecondPerSecond), 2) +
                                            Math.pow(accelerationZ.getValue().in(MetersPerSecondPerSecond), 2));
                // Correct step with new measurement
                targetPoseFilter.correct(
                    VecBuilder.fill(0, 0), // Input
                    VecBuilder.fill(
                        measurement.getX(),
                        measurement.getY()
                    ),
                    Matrix.eye(Nat.N2()).times(Math.pow(acceleration * 0.01, 2)) // Dynamically adjust measurement noise based on acceleration (more noise when accelerating)
                );
            }

        }
    }

    private Transform3d pose3dToTransform3d(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    private Pose3d lockToGround(Pose3d pose) {
        return new Pose3d(new Translation3d(pose.getX(), pose.getY(), 0),
                          new Rotation3d(0,0, pose.getRotation().getZ()));
    }

    private Pose3d getFilteredPose(double targetPoseYaw) {
        return new Pose3d(new Translation3d(targetPoseFilter.getXhat(0),
                                            targetPoseFilter.getXhat(1), 
                                            0),
                          new Rotation3d(0,
                                         0,
                                         targetPoseYaw));
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //Get the robot's current pose in the "field's" coordinate space. Note that we don't actually have
        //a true field-relative pose since we don't have a global reference. The 0,0 of the field space is just
        //wherever the robot starts. But we are only using the odometry for short-term pose estimation when
        //we lose sight of the tag, so it doen't really matter.
        Pose2d robotPose2DFieldSpace = drive.getPose();
        Pose3d robotPoseFieldSpace = new Pose3d(new Translation3d(robotPose2DFieldSpace.getX(), robotPose2DFieldSpace.getY(), 0),
                                                new Rotation3d(0, 0, robotPose2DFieldSpace.getRotation().getRadians()));
        
        if(LimelightHelpers.getHeartbeat(limelightName) == 0.0) {
            drive.stopWithX();
            return;
        }

        boolean hasTarget = LimelightHelpers.getTV(limelightName);
        Logger.recordOutput("FollowAprilTag/RobotPose_FieldSpace", robotPoseFieldSpace);
        Logger.recordOutput("FollowAprilTag/HasTarget", hasTarget);
        if(hasTarget)
        {        
            //Get the tag pose in the robot coordinate space from limelight.
            Pose3d tagPoseRobotSpace = LimelightHelpers.toPose3D(LimelightHelpers.getTargetPose_RobotSpace(limelightName));

            //This rotates the tag in robot space to follow the standard FRC convention of: X+ forward, Y+ left, Z+ up
            //instead of whatever limelight uses.
            //
            //The result is that the tag frame is X+ right, Y+ down, Z- Out of tag, when looking at the tag.
            //Limelight defines a left-handed frame for the tag frame with Z+ out, but since we use right-hand frames
            //this becomes Z-.
            //
            //This was only tested with a single camera position/orientation, it is possible this will break if the camera is moved.
            tagPoseRobotSpace = tagPoseRobotSpace.rotateBy(new Rotation3d(-Math.PI/2,0,-Math.PI/2));

            //Then we get the tag in the field space using the robot pose in field space
            tagPoseFieldSpace = robotPoseFieldSpace.plus(pose3dToTransform3d(tagPoseRobotSpace));

            //Use the tag pose to create a target pose in field space by adding an offset to it.
            Pose3d targetPoseFieldSpace = lockToGround(tagPoseFieldSpace.plus(targetOffset));

            //Filter this target pose to reduce jitter
            updateFilter(targetPoseFieldSpace); 
            filteredTargetPoseFieldSpace = getFilteredPose(targetPoseFieldSpace.getRotation().getZ());
            
            Logger.recordOutput("FollowAprilTag/TagPose_RobotSpace", tagPoseRobotSpace);
            Logger.recordOutput("FollowAprilTag/TagPose_FieldSpace", tagPoseFieldSpace);
            Logger.recordOutput("FollowAprilTag/TargetPose_FieldSpace", targetPoseFieldSpace);
            Logger.recordOutput("FollowAprilTag/FilteredTargetPose_FieldSpace", filteredTargetPoseFieldSpace);

            //This is just used for reference
            Pose3d targetPoseRobotSpace = tagPoseRobotSpace.plus(targetOffset);
            Logger.recordOutput("FollowAprilTag/TargetPose_RobotSpace", targetPoseRobotSpace);

            countSinceLastSeen = 0;
        } else {
            countSinceLastSeen++;

            // Run predict step even without measurement to maintain filter state
            updateFilter(null); // No measurement update, just predict
        }

        double xVelocityRequest = 0;
        double yVelocityRequest = 0;
        double rotationRequest = 0;

        //Set the XY Velocity if we have seen the tag within the count limit
        if (countSinceLastSeen < translationStopCount) { 
            //Calculate the velocity requests based on the difference between the current robot pose and the target pose
            xVelocityRequest = xPositionPID.calculate(robotPoseFieldSpace.getX(), filteredTargetPoseFieldSpace.getX());
            xVelocityRequest = MathUtil.clamp(xVelocityRequest, -maxSpeed, maxSpeed);

            yVelocityRequest = yPositionPID.calculate(robotPoseFieldSpace.getY(), filteredTargetPoseFieldSpace.getY());
            yVelocityRequest = MathUtil.clamp(yVelocityRequest, -maxSpeed, maxSpeed);            
        } else {
            //Else just stop the XY translation
            xVelocityRequest = 0;
            yVelocityRequest = 0;
            xPositionPID.reset(robotPoseFieldSpace.getX());
            yPositionPID.reset(robotPoseFieldSpace.getY());
            filterInitialized = false; //Reset the filter so that it will be re-initialized with the next measurement
        }

        //Set the rotation velocity if we have seen the tag within the count limit
        if (countSinceLastSeen < rotationStopCount) {
            //Get the target rotation in the field space
            Translation3d tagRobotDiffFieldSpace = tagPoseFieldSpace.getTranslation().minus(robotPoseFieldSpace.getTranslation());
            Rotation2d targetRotation = Rotation2d.fromRadians(Math.atan2(tagRobotDiffFieldSpace.getY(), 
                                                                          tagRobotDiffFieldSpace.getX())).
                                                                            plus(targetRobotRotationFacingTag);

            rotationRequest = rotationPID.calculate(robotPose2DFieldSpace.getRotation().getRadians(), targetRotation.getRadians());
            rotationRequest = MathUtil.clamp(rotationRequest, -maxAngularRate, maxAngularRate);
            
            Logger.recordOutput("FollowAprilTag/TargetRotation", targetRotation.getRadians());
        } else {
            rotationRequest = searchAngularRate;
            rotationPID.reset(robotPose2DFieldSpace.getRotation().getRadians());
            filterInitialized = false; //Reset the filter so that it will be re-initialized with the next measurement
        }
        Logger.recordOutput("FollowAprilTag/TranslationDeadband", translationDeadband);
        Logger.recordOutput("FollowAprilTag/RotationDeadband", rotationDeadband);

        xVelocityRequest = MathUtil.applyDeadband(xVelocityRequest, translationDeadband);
        yVelocityRequest = MathUtil.applyDeadband(yVelocityRequest, translationDeadband);
        rotationRequest = MathUtil.applyDeadband(rotationRequest, rotationDeadband);

        Logger.recordOutput("FollowAprilTag/XVelocityRequestFieldSpace", xVelocityRequest, MetersPerSecond);
        Logger.recordOutput("FollowAprilTag/YVelocityRequestFieldSpace", yVelocityRequest, MetersPerSecond);
        Logger.recordOutput("FollowAprilTag/RotationRequestFieldSpace", rotationRequest, RadiansPerSecond);

        ChassisSpeeds speeds =
            new ChassisSpeeds(
                xVelocityRequest,
                yVelocityRequest,
                rotationRequest);
        ChassisSpeeds commandedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation());
        Logger.recordOutput("FollowAprilTag/XVelocityRequestedRobotSpace", commandedSpeeds.vxMetersPerSecond, MetersPerSecond);
        Logger.recordOutput("FollowAprilTag/YVelocityRequestedRobotSpace", commandedSpeeds.vyMetersPerSecond, MetersPerSecond);
        Logger.recordOutput("FollowAprilTag/RotationRequestedRobotSpace", commandedSpeeds.omegaRadiansPerSecond, RadiansPerSecond);
        Logger.recordOutput("FollowAprilTag/EnableDrivetrain", enableDrivetrain);

        if(enableDrivetrain) {
            drive.runVelocity(commandedSpeeds);
        } else {
            drive.stopWithX();
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
