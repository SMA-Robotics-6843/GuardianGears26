package frc.robot.controls;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Meter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.Constants.DrivetrainConstants.*;
//import swervelib.SwerveInputStream;
import java.util.Set;
public class DriverControls {

private static PathConstraints constraints = new PathConstraints(
                        3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));


  private static Pose2d getTargetPose() {
    Pose2d hubPose = new Pose2d(
        Meter.of(11.902),
        Meter.of(4.031),
        Rotation2d.kZero);

    Logger.recordOutput("controller/TargetHubPose", hubPose);

    return hubPose;
  }

  public static void configure(int port, SwerveDriveSubsystem drivetrain, Superstructure superstructure, VisionSubsystem vision) {
    CommandXboxController controller = new CommandXboxController(ControllerConstants.kDriverControllerPort);
    var driveRequest = new SwerveRequest.FieldCentric();
         // Note that X is defined as forward according to WPILib convention,
          // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> driveFieldCentric
                // Drive forward with negative Y (forward)
                .withVelocityX(-controller.getLeftY() * MaxSpeed)
                // Drive left with negative X (left)
                .withVelocityY(-controller.getLeftX() * MaxSpeed)
                // Drive counterclockwise with negative X (left)
                                .withRotationalRate(
                                                -controller.getRightX() * MaxAngularRate)));



                // driverController.leftBumper().whileTrue(drivetrain.applyRequest(
                //                 () -> point.withModuleDirection(
                //                                 new Rotation2d(-driverController.getLeftY(),
                //                                                 -driverController.getLeftX()))));


                /*controller.leftBumper().whileTrue(drivetrain.applyRequest(
=======
              
               controller.x().onTrue(superstructure.intakeCommand());
               controller.y().onTrue(superstructure.ejectCommand());
                controller.leftBumper().whileTrue(drivetrain.applyRequest(
>>>>>>> 7d5b3a6414efe3472a43bfd74da04911d1856b08
                        () -> driveRobotCentric
                                .withVelocityX(-controller.getLeftY() * MaxSpeed)
                                .withVelocityY(-controller.getLeftX() * MaxSpeed)
                                .withRotationalRate(-controller.getRightX() * MaxAngularRate))); */
                

                controller.rightBumper().whileTrue(drivetrain.applyRequest(
                        () -> driveFieldCentric
                                .withVelocityX((-controller.getLeftY() * MaxSpeed) / 5)
                                .withVelocityY((-controller.getLeftX() * MaxSpeed) / 5)
                                .withRotationalRate(-controller.getRightX() * MaxAngularRate)));

                controller.rightBumper().and(controller.leftBumper()).whileTrue(drivetrain.applyRequest(
                        () -> driveRobotCentric
                                .withVelocityX((-controller.getLeftY() * MaxSpeed) / 5)
                                .withVelocityY((-controller.getLeftX() * MaxSpeed) / 5)
                                .withRotationalRate(-controller.getRightX() * MaxAngularRate)));

                // reset the field-centric heading on right stick press
                controller.rightStick().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));



                controller.leftTrigger().and(controller.rightTrigger().negate())
                        .whileTrue(Commands.defer(
                                () -> AutoBuilder.pathfindToPose(
                                        vision.decidePoseAlignmentLeft(),
                                        constraints, 0),
                                Set.of(drivetrain)));

                controller.rightTrigger().and(controller.leftTrigger().negate())
                        .whileTrue(Commands.defer(
                                () -> AutoBuilder.pathfindToPose(
                                        vision.decidePoseAlignmentRight(),
                                        constraints, 0),
                                Set.of(drivetrain)));

                // TODO: Run sysid
                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                controller.back().and(controller.y())
                        .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                controller.back().and(controller.x())
                        .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                controller.start().and(controller.y())
                        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                controller.start().and(controller.x())
                        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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
          drivetrain.getState().Speeds,
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
    });};

    
  };
 