package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
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
import edu.wpi.first.units.measure.Angle;
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

public class IntakePivotSubsystem extends SubsystemBase {


  // 5:1, 5:1, 60/18 reduction
  private SmartMotorControllerConfig intakePivotSmartMotorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(3.25, 0, .6, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(360))
      //.withFeedforward(new SimpleMotorFeedforward(0, 10, 0))
      .withFeedforward(new ArmFeedforward(0,.0799, 8.25, 0.4))
      .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 60.0 / 18.0)))
      // .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 60.0 /
      // 18.0, 42)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withSoftLimit(Degrees.of(-40), Degrees.of(150))
      .withStatorCurrentLimit(Amps.of(30))
      .withClosedLoopRampRate(Seconds.of(0.1));
   

  public SparkMax IntakeMotor = new SparkMax(Constants.IntakeConstants.kExtendMotorId, MotorType.kBrushless);

  private SmartMotorController intakePivotController = new SparkWrapper(IntakeMotor, DCMotor.getNEO(1),
      intakePivotSmartMotorConfig);

  private final ArmConfig intakePivotConfig = new ArmConfig(intakePivotController)
      .withSoftLimits(Degrees.of(-30), Degrees.of(150))
      .withHardLimit(Degrees.of(-40), Degrees.of(155))
      .withStartingPosition(Degrees.of(-20))
      .withLength(Feet.of(1))
      .withMass(Pounds.of(2)) // Reis says: 2 pounds, not a lot
      .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

  private Arm intakePivot = new Arm(intakePivotConfig);

  public IntakePivotSubsystem() {
    // pivotMotor.factoryReset();
  }


  public Command setPivotAngle(Angle angle) {
    return intakePivot.setAngle(angle).withName("IntakePivot.SetAngle");
  }

  public Command rezero() {
    return Commands.runOnce(() -> IntakeMotor.getEncoder().setPosition(0), this).withName("IntakePivot.Rezero");
  }

  /**
   * Command to deploy intake and run roller while held.
   * Stops roller when released.
   */
  /* 
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
*/
  private void setIntakeStow() {
    intakePivotController.setPosition(Degrees.of(0));
  }

  private void setIntakeFeed() {
    intakePivotController.setPosition(Degrees.of(59));
  }

  private void setIntakeHold() {
    intakePivotController.setPosition(Degrees.of(115));
  }

  private void setIntakeDeployed() {
    intakePivotController.setPosition(Degrees.of(148));
  }

  @Override
  public void periodic() {
    intakePivot.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    intakePivot.simIterate();
  }
}
