package frc.robot.controls;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Meter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveDriveSubsystem;
//import swervelib.SwerveInputStream;

public class DriverControls {

  private static Pose2d getTargetPose() {
    Pose2d hubPose = new Pose2d(
        Meter.of(11.902),
        Meter.of(4.031),
        Rotation2d.kZero);

    Logger.recordOutput("DriverControls/TargetHubPose", hubPose);

    return hubPose;
  }

  public static void configure(int port, SwerveDriveSubsystem drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);

    var driveRequest = new SwerveRequest.FieldCentric();
  
    drivetrain.setControl(driveRequest.withVelocityX(-controller.getLeftY() * DrivetrainConstants.MaxSpeed)
                                  .withVelocityY(-controller.getLeftX() * DrivetrainConstants.MaxSpeed)
                                  .withRotationalRate(-controller.getRightX() * DrivetrainConstants.MaxAngularRate));


    // controller.rightBumper().whileTrue(Commands.run(
    // () -> {
    // driveInputStream
    // .aim(getTargetPose())
    // .aimWhile(true);
    // }).finallyDo(() -> driveInputStream.aimWhile(false)));

    //drivetrain.setDefaultCommand(
    //    drivetrain.driveFieldOriented(driveInputStream).withName("Drive" + ".test"));

    // NOTE: YAGSL way of doing direct drive to pose
    // driveInputStream.driveToPose(drivetrain.getTargetPoseSupplier(),
    // new ProfiledPIDController(5, 0, 0,
    // new Constraints(5, 2)),
    // new ProfiledPIDController(5, 0, 0,
    // new Constraints(
    // Units.degreesToRadians(360),
    // Units.degreesToRadians(180))));

    // controller.rightBumper().whileTrue(Commands.runEnd(
    // () -> driveInputStream.driveToPoseEnabled(true),
    // () -> driveInputStream.driveToPoseEnabled(false)));

    // NOTE: PathPlanner way of doing obstacle-aware drive to pose
    // controller.rightBumper()
    // .whileTrue(Commands.defer(
    // () -> drivetrain.driveToPose(drivetrain.getTargetPose()),
    // java.util.Set.of(drivetrain)));
/* 
    if (DriverStation.isTest()) {
      // drivetrain.setDefaultCommand(driveFieldOrientedAngularVelocity);
      // Overrides drive command above!
      // Might be useful for robot-oriented controls in testing

      controller.b().whileTrue(drivetrain.centerModulesCommand());
      controller.x().whileTrue(Commands.runOnce(drivetrain::lock, drivetrain).repeatedly());
      controller.y().onTrue((Commands.runOnce(drivetrain::zeroGyro)));

      controller.start().whileTrue(drivetrain.sysIdAngleMotorCommand());
      controller.back().whileTrue(drivetrain.sysIdDriveMotorCommand());
    } else if (Robot.isSimulation()) {
      // Fire fuel 10 times per second while button is held
      controller.back().whileTrue(
          Commands.repeatingSequence(
              fireFuel(drivetrain, superstructure),
              Commands.waitSeconds(0.1)));
    } else {
      controller.start().onTrue((Commands.runOnce(drivetrain::zeroGyro)));

      controller.leftBumper().whileTrue(
          superstructure.feedAllCommand()
              .finallyDo(() -> superstructure.stopFeedingAllCommand().schedule()));

      controller.rightBumper()
          .whileTrue(superstructure.setIntakeDeployAndRoll().withName("OperatorControls.intakeDeployed"));

    }*/
  }

  public static Command fireFuel(SwerveDriveSubsystem drivetrain, Superstructure superstructure) {
    return Commands.runOnce(() -> {
      SimulatedArena arena = SimulatedArena.getInstance();

      GamePieceProjectile fuel = new RebuiltFuelOnFly(
          drivetrain.getPose().getTranslation(),
          new Translation2d(
              superstructure.turret.turretTranslation.getX() * -1,
              superstructure.turret.turretTranslation.getY()),
          drivetrain.getSwerveDrive().getFieldRelativeSpeed(),
          drivetrain.getPose().getRotation().rotateBy(superstructure.getAimRotation3d().toRotation2d()),
          superstructure.turret.turretTranslation.getMeasureZ(),

          // 0.5 times because we're applying spin to the fuel as we shoot it
          superstructure.getTangentialVelocity().times(0.5),
          superstructure.getHoodAngle());

      // Configure callbacks to visualize the flight trajectory of the projectile
      fuel.withProjectileTrajectoryDisplayCallBack(
          // Callback for when the note will eventually hit the target (if configured)
          (pose3ds) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileSuccessfulShot",
              pose3ds.toArray(Pose3d[]::new)),
          // Callback for when the note will eventually miss the target, or if no target
          // is configured
          (pose3ds) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileUnsuccessfulShot",
              pose3ds.toArray(Pose3d[]::new)));

      arena.addGamePieceProjectile(fuel);
    });
  }
}
