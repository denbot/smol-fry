// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(0);
    private final Drive drive;

    public RobotContainer() {
        switch (Constants.currentMode) {
        case REAL:
            // Real robot, instantiate hardware IO implementations
            // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
            // a CANcoder
            drive =
                new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFX(TunerConstants.FrontLeft),
                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                    new ModuleIOTalonFX(TunerConstants.BackRight));
            break;

        case SIM:
            // Sim robot, instantiate physics sim IO implementations
            drive =
                new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(TunerConstants.FrontLeft),
                    new ModuleIOSim(TunerConstants.FrontRight),
                    new ModuleIOSim(TunerConstants.BackLeft),
                    new ModuleIOSim(TunerConstants.BackRight));
            break;

        default:
            // Replayed robot, disable IO implementations
            drive =
                new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});
            break;
        }

        configureBindings();
    }

    private void configureBindings() {
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));

        // Lock to 0° when right stick button is held
        controller.rightStick()
            .whileTrue(DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () -> Rotation2d.kZero));
        
        // Reset gyro to 0° when start button is pressed
        controller.start()
            .onTrue(Commands.runOnce(() -> 
                    drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),drive)
            .ignoringDisable(true));
    }

    public Pose2d getRobotPosition(){
        return drive.getPose();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
