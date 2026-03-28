package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeSubsystem extends SubsystemBase {

  private static final double INTAKE_SPEED = 1.0;

  // ThriftyNova controlling the intake roller
  private SparkMax rollerSpark = new SparkMax(Constants.IntakeConstants.kIntakeMotorId, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.LOW)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1))) // Direct drive, adjust if geared
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorController smc = new SparkWrapper(rollerSpark, DCMotor.getNEO(1), smcConfig);
  
  private final FlyWheelConfig intakeConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("IntakeRoller", TelemetryVerbosity.LOW);

  private FlyWheel intake = new FlyWheel(intakeConfig);

  // 5:1, 5:1, 60/18 reduction
  private  SmartMotorControllerConfig intakePivotSmartMotorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(25, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(360))
      .withFeedforward(new SimpleMotorFeedforward(0, 10, 0))
      .withTelemetry("IntakePivotMotor", TelemetryVerbosity.LOW)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 60.0 / 18.0)))
      // .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 60.0 /
      // 18.0, 42)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      //.withSoftLimit(Degrees.of(0), Degrees.of(150))
      .withStatorCurrentLimit(Amps.of(30));
      //.withClosedLoopRampRate(Seconds.of(0.1))
      //.withOpenLoopRampRate(Seconds.of(0.1));

  public static SparkMax IntakeMotor = new SparkMax(Constants.IntakeConstants.kExtendMotorId, MotorType.kBrushless);

  private SmartMotorController intakePivotController = new SparkWrapper(IntakeMotor, DCMotor.getNEO(1),
      intakePivotSmartMotorConfig);

  private final ArmConfig intakePivotConfig = new ArmConfig(intakePivotController)
      //.withSoftLimits(Degrees.of(0), Degrees.of(170))
      .withHardLimit(Degrees.of(0), Degrees.of(180))
      .withStartingPosition(Degrees.of(0))
      .withLength(Feet.of(1))
      .withMass(Pounds.of(2)) // Reis says: 2 pounds, not a lot
      .withTelemetry("IntakePivot", TelemetryVerbosity.LOW);

  private Arm intakePivot = new Arm(intakePivotConfig);

  private PIDController pivotPID = new PIDController(2, 0, 0);

  public IntakeSubsystem() {
    this.setDefaultCommand(run(()->IntakeMotor.set(0)));
    pivotPID.setTolerance(.02);
    // pivotMotor.factoryReset();
  }

  /**
   * Command to run the intake while held.
   */
  public Command intakeCommand() {
    return intake.set(INTAKE_SPEED).withName("Intake.Run");
  }

    /**
   * Command to run the intake while held.
   */
  public Command intakeStopCommand() {
    return intake.set(0).withName("Intake.Stop");
  }

  /**
   * Command to eject while held.
   */
  public Command ejectCommand() {
    return intake.set(-INTAKE_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Intake.Eject");
  }

  public Command setPivotAngle(Angle angle) {
    return intakePivot.setAngle(angle).withName("IntakePivot.SetAngle");
  }

  public Command intakeArmDown () {
   return run(()->{
      IntakeMotor.set(0.3);
    });
  };

  public Command intakeArmUp () {
   return run(()->{
      IntakeMotor.set(-0.3);
      System.out.println ("intake up works");
    });
  };

  public Command rezero() {
    return Commands.runOnce(() -> IntakeMotor.getEncoder().setPosition(0), this).withName("IntakePivot.Rezero");
  }

  /**
   * Command to deploy intake and run roller while held.
   * Stops roller when released.
   */
  public Command deployAndRollCommand() {
    return Commands.run(() -> {
      setIntakeDeployed();
      smc.setDutyCycle(INTAKE_SPEED);
    }, this).finallyDo(() -> {
      smc.setDutyCycle(0);
      setIntakeHold();
    }).withName("Intake.DeployAndRoll");
  }

  public Command backFeedAndRollCommand() {
    return Commands.run(() -> {
      setIntakeDeployed();
      // smc.setDutyCycle(-INTAKE_SPEED);
    }, this).finallyDo(() -> {
      smc.setDutyCycle(0);
      setIntakeHold();
    }).withName("Intake.BackFeedAndRoll");
  }

  public Command setIntakeStow() {
    return Commands.run(() -> {
    IntakeMotor.set(pivotPID.calculate(IntakeMotor.getAbsoluteEncoder().getPosition(), .22));
    SmartDashboard.putNumber("intake PID output", pivotPID.calculate(IntakeMotor.getAbsoluteEncoder().getPosition(), .22));
    System.out.println("intake stow");
  });
    
  }

  public Command setIntakeHold() {
    return Commands.run(() -> {
    IntakeMotor.set(pivotPID.calculate(IntakeMotor.getAbsoluteEncoder().getPosition(), .39));
    System.out.println("intake hold");
  });
  } 

  public Command setIntakeDeployed() {
   return Commands.run(() -> {
    IntakeMotor.set(pivotPID.calculate(IntakeMotor.getAbsoluteEncoder().getPosition(), .68));
    System.out.println("intake deploy");
    });
  }

  @Override
  public void periodic() {
    intake.updateTelemetry();
    intakePivot.updateTelemetry();
    SmartDashboard.putNumber("intakePivot Encoder", IntakeMotor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putBoolean("intake at setpoint", pivotPID.atSetpoint());
  }

  @Override
  public void simulationPeriodic() {
    intake.simIterate();
    intakePivot.simIterate();
  }
}
