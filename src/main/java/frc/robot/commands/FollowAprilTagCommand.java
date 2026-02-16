package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;

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

    // Desired offset from the tag
    private final Transform3d targetOffset = new Transform3d(new Translation3d(1.5, 0.0, 0.0), new Rotation3d(0, 0, 0));
    private ProfiledPIDController  xPositionPID;
    private ProfiledPIDController  yPositionPID;
    private ProfiledPIDController  rotationPID;

    // The desired robot rotation relative to the tag (facing the tag)
    private final Rotation2d targetRobotRotationFacingTag = Rotation2d.fromDegrees(0); 

    // Kalman filter parameters
    private static final double dtSeconds = 0.02; // 20ms loop time
    private static final double modelStdDev = 0.3; // process noise
    private static final double measurementStdDev = 0.15; // Measurement noise from vision
    
    private KalmanFilter<N3, N3, N3> targetPoseFilter;
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
        // State: [x, y, theta], Input: [delta_x, delta_y, delta_theta] (none actually used), Output: [x, y, theta]
        // A = 0 (The state does not change based on the previous state since we are not tracking velocity)
        // B = I (identity - for prediction step)
        // C = I (identity - we observe the full state)
        // D = 0 (no feedthrough)
        
        Matrix<N3, N3> A = new Matrix<>(Nat.N3(), Nat.N3());
        Matrix<N3, N3> B = Matrix.eye(Nat.N3());
        Matrix<N3, N3> C = Matrix.eye(Nat.N3());
        Matrix<N3, N3> D = new Matrix<>(Nat.N3(), Nat.N3());
        
        LinearSystem<N3, N3, N3> system = new LinearSystem<>(A, B, C, D);
        
        targetPoseFilter = new KalmanFilter<>(
            Nat.N3(),
            Nat.N3(),
            system,
            VecBuilder.fill(modelStdDev, modelStdDev, modelStdDev), // Model state std devs
            VecBuilder.fill(measurementStdDev, measurementStdDev, measurementStdDev), // Measurement std devs
            dtSeconds
        );        
    }

    private void updateFilter(Pose3d measurement) {
        //Filter the tag pose in field space
        if(!filterInitialized) {
            if(measurement != null) {
                targetPoseFilter.setXhat(VecBuilder.fill(
                    measurement.getX(),
                    measurement.getY(),
                    measurement.getZ()
                    ));
                filterInitialized = true;
            }
        } else {
            // Predict step (Assuming the tag is stationary, so input is zero)
            targetPoseFilter.predict(VecBuilder.fill(0, 0, 0), dtSeconds);
            
            if(measurement != null) {

                double acceleration = Math.sqrt(Math.pow(accelerationX.getValue().in(MetersPerSecondPerSecond), 2) +
                                            Math.pow(accelerationY.getValue().in(MetersPerSecondPerSecond), 2) +
                                            Math.pow(accelerationZ.getValue().in(MetersPerSecondPerSecond), 2));
                // Correct step with new measurement
                targetPoseFilter.correct(
                    VecBuilder.fill(0, 0, 0), // Input
                    VecBuilder.fill(
                        measurement.getX(),
                        measurement.getY(),
                        measurement.getZ()
                    ),
                    Matrix.eye(Nat.N3()).times(Math.pow(acceleration * 0.01, 2)) // Dynamically adjust measurement noise based on acceleration (more noise when accelerating)
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
        return new Pose3d( new Translation3d(targetPoseFilter.getXhat(0),
                                            targetPoseFilter.getXhat(1),
                                            targetPoseFilter.getXhat(2)),
            new Rotation3d(0,
                            0,
                            targetPoseYaw)
        );
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
            //Get the tag pose in the robot coordinate space from limelight and then
            //use the robot pose to get that tag in field space

            //This is all very hacky for a demo.
            //I don't know what the heck limelight is doing with coordinate spaces.
            //From testing it seems that it does not follow the standard FRC convention for robot coordinate space (or what is listed on the limelight website).
            //Also the coordiante frame of april tags seems to be left-handed...
            //I feel like I'm doing something wrong, but this mess puts the tag in robot space with the expected coordinate frame
            //and it follows the PhotoVision frame for tags, which is not left-handed...
            double[] tagPoseRobotSpaceArray = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
            Pose3d tagPoseRobotSpace = new Pose3d(new Translation3d(tagPoseRobotSpaceArray[2], -tagPoseRobotSpaceArray[0], -tagPoseRobotSpaceArray[1]), 
                                                  new Rotation3d(-Math.toRadians(tagPoseRobotSpaceArray[5]), Math.toRadians(tagPoseRobotSpaceArray[3]), -Math.toRadians(tagPoseRobotSpaceArray[4] + 180)));
            tagPoseFieldSpace = robotPoseFieldSpace.plus(pose3dToTransform3d(tagPoseRobotSpace));
            Logger.recordOutput("FollowAprilTag/TagPose_RobotSpace", tagPoseRobotSpace);
            Logger.recordOutput("FollowAprilTag/TagPose_FieldSpace", tagPoseFieldSpace);

            //Use the tag pose to create a filtered target pose in field space by adding the target offset to the filtered tag pose in field space.
            Pose3d targetPoseFieldSpace = lockToGround(tagPoseFieldSpace.plus(targetOffset));
            updateFilter(targetPoseFieldSpace);
            filteredTargetPoseFieldSpace = getFilteredPose(targetPoseFieldSpace.getRotation().getZ());

            //Update target pose to be something actually achievable by the robot
            //This zeros the Z translation, roll, and pitch.
            filteredTargetPoseFieldSpace = lockToGround(filteredTargetPoseFieldSpace);

            Logger.recordOutput("FollowAprilTag/TargetPose_FieldSpace", targetPoseFieldSpace);
            Logger.recordOutput("FollowAprilTag/FilteredTargetPose_FieldSpace", filteredTargetPoseFieldSpace);

            //Currently this is just used for reference
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

        //xVelocityRequest = 0;
        //yVelocityRequest = 0;
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

        //Only command the drivetrain if the boolean is set to enable it
        //This just allows easy disabiling of the drivetrain for safer testing.
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
