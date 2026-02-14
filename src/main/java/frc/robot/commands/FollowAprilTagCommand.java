package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;

public class FollowAprilTagCommand extends Command {
    private final boolean enableDrivetrain = false;
    private final double maxSpeed = Meters.of(0.5).in(Meters);
    private final double maxAngularRate = RotationsPerSecond.of(2.0).in(RadiansPerSecond);
    private final double searchAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond);

    private final double rotationDeadband = maxAngularRate * 0.0;
    private final double translationDeadband = maxSpeed * 0.0;

    private final String limelightName = "limelight";
    
    // Desired offset from the tag (2 meters in front)
    private final Transform3d targetOffset = new Transform3d(new Translation3d(2.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
    private final PIDController xPositionPID = new PIDController(5.0, 0, 0);
    private final PIDController yPositionPID = new PIDController(5.0, 0, 0);
    private final PIDController rotationPID = new PIDController(5.0, 0, 0);

    // The desired robot rotation relative to the tag (facing the tag)
    private final Rotation2d targetRobotRotationFacingTag = Rotation2d.fromDegrees(180); 

    // Kalman filter parameters
    private static final double dtSeconds = 0.02; // 20ms loop time
    private static final double modelStdDevX = 0.5; // process noise
    private static final double modelStdDevY = 0.5;
    private static final double modelStdDevZ = 0.5;
    private static final double modelStdDevTheta = 0.5;
    private static final double measurementStdDevX = 0.05; // Measurement noise from vision
    private static final double measurementStdDevY = 0.05;
    private static final double measurementStdDevZ = 0.05;
    private static final double measurementStdDevTheta = 0.1;
    
    private KalmanFilter<N6, N6, N6> tagPoseFilter;
    private boolean filterInitialized = false;

    private Drive drive;

    private int countSinceLastSeen = 0;

    private Pose3d filteredTargetPoseFieldSpace = new Pose3d();
    private Pose3d filteredTagPoseFieldSpace = new Pose3d();

    public FollowAprilTagCommand(Drive drive) {
        addRequirements(drive);
        this.drive = drive;
        createFilter();
    }

    private void createFilter() {
        // State: [x, y, theta], Input: [delta_x, delta_y, delta_theta] (none actually used), Output: [x, y, theta]
        // A = 0 (The state does not change based on the previous state since we are not tracking velocity)
        // B = I (identity - for prediction step)
        // C = I (identity - we observe the full state)
        // D = 0 (no feedthrough)
        
        Matrix<N6, N6> A = new Matrix<>(Nat.N6(), Nat.N6());
        Matrix<N6, N6> B = Matrix.eye(Nat.N6());
        Matrix<N6, N6> C = Matrix.eye(Nat.N6());
        Matrix<N6, N6> D = new Matrix<>(Nat.N6(), Nat.N6());
        
        LinearSystem<N6, N6, N6> system = new LinearSystem<>(A, B, C, D);
        
        tagPoseFilter = new KalmanFilter<>(
            Nat.N6(),
            Nat.N6(),
            system,
            VecBuilder.fill(modelStdDevX, modelStdDevY, modelStdDevZ, modelStdDevTheta, modelStdDevTheta, modelStdDevTheta), // Model state std devs
            VecBuilder.fill(measurementStdDevX, measurementStdDevY, measurementStdDevZ, measurementStdDevTheta, measurementStdDevTheta, measurementStdDevTheta), // Measurement std devs
            dtSeconds
        );        
    }

    private void updateFilter(Pose3d measurement) {
        //Filter the tag pose in field space
        if(!filterInitialized) {
            if(measurement != null) {
                tagPoseFilter.setXhat(VecBuilder.fill(
                    measurement.getX(),
                    measurement.getY(),
                    measurement.getZ(),
                    measurement.getRotation().getX(),
                    measurement.getRotation().getY(),
                    measurement.getRotation().getZ()
                    ));
                filterInitialized = true;
            }
        } else {
            // Predict step (Assuming the tag is stationary, so input is zero)
            tagPoseFilter.predict(VecBuilder.fill(0, 0, 0,0,0,0), dtSeconds);
            
            if(measurement != null) {
                // Correct step with new measurement
                tagPoseFilter.correct(
                    VecBuilder.fill(0, 0, 0,0,0,0), // Input
                    VecBuilder.fill(
                        measurement.getX(),
                        measurement.getY(),
                        measurement.getZ(),
                        measurement.getRotation().getX(),
                        measurement.getRotation().getY(),
                        measurement.getRotation().getZ()

                    )
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

    private Pose3d getFilteredPose() {
        return new Pose3d( new Translation3d(tagPoseFilter.getXhat(0),
                                            tagPoseFilter.getXhat(1),
                                            tagPoseFilter.getXhat(2)),
            new Rotation3d(tagPoseFilter.getXhat(3),
                            tagPoseFilter.getXhat(4),
                            tagPoseFilter.getXhat(5))
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
        
        
        boolean hasTarget = LimelightHelpers.getTV(limelightName);
        Logger.recordOutput("FollowAprilTag/RobotPose_FieldSpace", robotPoseFieldSpace);
        Logger.recordOutput("FollowAprilTag/HasTarget", hasTarget);
        if(hasTarget)
        {        
            //Get the tag pose in the robot coordinate space from limelight and then
            //use the robot pose to get that tag in field space

            //I don't know what the heck limelight is doing with coordinate spaces.
            //From testing it seems that it does not follow the standard FRC convention for robot coordinate space (or what is listed on the limelight website).
            //Also the coordiante frame of april tags seems to be left-handed...
            //I feel like I'm doing something wrong, but this mess puts the tag in robot space with the expected coordinate frame
            //and it follows the PhotoVision frame for tags, which is not left-handed...
            double[] tagPoseRobotSpaceArray = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
            Pose3d tagPoseRobotSpace = new Pose3d(new Translation3d(tagPoseRobotSpaceArray[2], tagPoseRobotSpaceArray[0], -tagPoseRobotSpaceArray[1]), 
                                                  new Rotation3d(Math.toRadians(tagPoseRobotSpaceArray[5]), Math.toRadians(-tagPoseRobotSpaceArray[3]), Math.toRadians(tagPoseRobotSpaceArray[4] + 180)));
            Pose3d tagPoseFieldSpace = robotPoseFieldSpace.plus(pose3dToTransform3d(tagPoseRobotSpace));
            Logger.recordOutput("FollowAprilTag/TagPose_RobotSpace", tagPoseRobotSpace);
            Logger.recordOutput("FollowAprilTag/TagPose_FieldSpace", tagPoseFieldSpace);

            updateFilter(tagPoseFieldSpace);
            filteredTagPoseFieldSpace = getFilteredPose();
            Logger.recordOutput("FollowAprilTag/FilteredTagPose_FieldSpace", filteredTagPoseFieldSpace);

            //Use the filtered tag pose to create a filtered target pose in field space by adding the target offset to the filtered tag pose in field space.
            filteredTargetPoseFieldSpace = filteredTagPoseFieldSpace.plus(targetOffset);

            //Update target pose to be something actually achievable by the robot
            //This zeros the Z translation, roll, and pitch.
            filteredTargetPoseFieldSpace = lockToGround(filteredTargetPoseFieldSpace);

            Logger.recordOutput("FollowAprilTag/FilteredTargetPose_FieldSpace", filteredTargetPoseFieldSpace);


            //Currently these are just used for reference
            Pose3d targetPoseRobotSpace = tagPoseRobotSpace.plus(targetOffset);
            Pose3d targetPoseFieldSpace = lockToGround(tagPoseFieldSpace.plus(targetOffset));
            Logger.recordOutput("FollowAprilTag/TargetPose_RobotSpace", targetPoseRobotSpace);
            Logger.recordOutput("FollowAprilTag/TargetPose_FieldSpace", targetPoseFieldSpace);

            countSinceLastSeen = 0;
        } else {
            countSinceLastSeen++;

            // Run predict step even without measurement to maintain filter state
            updateFilter(null); // No measurement update, just predict

            filteredTagPoseFieldSpace = getFilteredPose();
            filteredTargetPoseFieldSpace = lockToGround(filteredTagPoseFieldSpace.plus(targetOffset));

            Logger.recordOutput("FollowAprilTag/FilteredTargetPose_FieldSpace", filteredTargetPoseFieldSpace);
            Logger.recordOutput("FollowAprilTag/FilteredTagPose_FieldSpace", filteredTagPoseFieldSpace);
        }

        double xVelocityRequest = 0;
        double yVelocityRequest = 0;
        double rotationRequest = 0;

        if (countSinceLastSeen < 3 && filterInitialized) { //If we recently saw the tag, keep trying to go to the last known position
            //Get the XY and rotation error between where the robot is and where we want it to be.

            //Field relative
            double adjustedTargetX = filteredTargetPoseFieldSpace.getX() - robotPoseFieldSpace.getX();
            double adjustedTargetY = filteredTargetPoseFieldSpace.getY() - robotPoseFieldSpace.getY();


            Pose3d filteredTagRobotSpace = filteredTagPoseFieldSpace.relativeTo(robotPoseFieldSpace);

            //This could be changed to just face the camera towards the tag, but
            //what if there are multiple cameras? 
            Rotation2d targetRotation = Rotation2d.fromRadians(Math.atan2(filteredTagRobotSpace.getY(), 
                                                                          filteredTagRobotSpace.getX())).
                                                                            plus(targetRobotRotationFacingTag);
            
            //Robot relative
            //double adjustedTargetX = targetPoseRobotSpace.getX();
            //double adjustedTargetY = targetPoseRobotSpace.getY();
            //double targetRotation = Math.atan2(tagPoseRobotSpace.getY(), tagPoseRobotSpace.getX());

            //Calculate the velocity requests based on that error.
            xVelocityRequest = xPositionPID.calculate(robotPoseFieldSpace.getX(), filteredTargetPoseFieldSpace.getX());
            xVelocityRequest = MathUtil.clamp(xVelocityRequest, -maxSpeed, maxSpeed);

            yVelocityRequest = yPositionPID.calculate(robotPoseFieldSpace.getY(), filteredTargetPoseFieldSpace.getY());
            yVelocityRequest = MathUtil.clamp(yVelocityRequest, -maxSpeed, maxSpeed);

            rotationRequest = rotationPID.calculate(0, targetRotation.getRadians());
            rotationRequest = MathUtil.clamp(rotationRequest, -maxAngularRate, maxAngularRate);
            
            Logger.recordOutput("FollowAprilTag/AdjustedTargetX", adjustedTargetX, Meters);
            Logger.recordOutput("FollowAprilTag/AdjustedTargetY", adjustedTargetY, Meters);
            Logger.recordOutput("FollowAprilTag/TargetRotation", targetRotation.getDegrees());

            countSinceLastSeen++;
        } else {

            xVelocityRequest = 0;
            yVelocityRequest = 0;
            rotationRequest = searchAngularRate;
            xPositionPID.reset();
            yPositionPID.reset();
            rotationPID.reset();
            filterInitialized = false; //Reset the filter so that it will be re-initialized with the next measurement
        }

       // xVelocityRequest = MathUtil.applyDeadband(xVelocityRequest, translationDeadband);
       // yVelocityRequest = MathUtil.applyDeadband(yVelocityRequest, translationDeadband);
       // rotationRequest = MathUtil.applyDeadband(rotationRequest, rotationDeadband);

        Logger.recordOutput("FollowAprilTag/XVelocityRequest", xVelocityRequest, MetersPerSecond);
        Logger.recordOutput("FollowAprilTag/YVelocityRequest", yVelocityRequest, MetersPerSecond);
        Logger.recordOutput("FollowAprilTag/RotationRequest", rotationRequest, RadiansPerSecond);

        //Only command the drivetrain if the boolean is set to enable it
        //This just allows easy disabiling of the drivetrain for safer testing.
        if(enableDrivetrain) {
            ChassisSpeeds speeds =
                new ChassisSpeeds(
                    xVelocityRequest,
                    yVelocityRequest,
                    rotationRequest);
                    
            ChassisSpeeds commandedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation());
            Logger.recordOutput("FollowAprilTag/XVelocityCommanded", commandedSpeeds.vxMetersPerSecond, MetersPerSecond);
            Logger.recordOutput("FollowAprilTag/YVelocityCommanded", commandedSpeeds.vyMetersPerSecond, MetersPerSecond);
            Logger.recordOutput("FollowAprilTag/RotationCommanded", commandedSpeeds.omegaRadiansPerSecond, RadiansPerSecond);
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
