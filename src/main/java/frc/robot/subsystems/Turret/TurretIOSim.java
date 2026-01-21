package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretIOSim implements TurretIO {
  private final DCMotor _turnMotor = DCMotor.getKrakenX44Foc(1);
  private final SingleJointedArmSim turnSim =
      new SingleJointedArmSim(_turnMotor, 35, 0.001, 0.1, -Math.PI, Math.PI, false, 0, 0.0, 0.0);

  private final DCMotor _hoodMotor = DCMotor.getKrakenX44Foc(1);
  private final SingleJointedArmSim hoodSim =
      new SingleJointedArmSim(_hoodMotor, 2, 0.005, 0.1, 0, Math.PI, false, 0, 0.002, 0.0002);

  private final DCMotor _flywheelMotor = DCMotor.getKrakenX60Foc(1);
  private final FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(_flywheelMotor, 0.005, 2), _flywheelMotor, 0.005);

  private final DCMotor _shootMotor = DCMotor.getKrakenX60Foc(1);
  private final FlywheelSim shootSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(_shootMotor, 0.005, 2), _shootMotor, 0.005);

  private Voltage turnAppliedVoltage = Volts.zero();
  private Voltage hoodAppliedVoltage = Volts.zero();
  private Voltage flywheelAppliedVoltage = Volts.zero();
  private Voltage shootAppliedVoltage = Volts.zero();

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    turnSim.setInput(turnAppliedVoltage.in(Volts));
    hoodSim.setInput(hoodAppliedVoltage.in(Volts));
    flywheelSim.setInput(flywheelAppliedVoltage.in(Volts));
    shootSim.setInput(shootAppliedVoltage.in(Volts));

    turnSim.update(0.02);
    hoodSim.update(0.02);
    flywheelSim.update(0.02);
    shootSim.update(0.02);

    inputs.turnPosition = Radians.of(turnSim.getAngleRads());
    inputs.turnVelocity = RadiansPerSecond.of(turnSim.getVelocityRadPerSec());
    inputs.turnAppliedVoltage = turnAppliedVoltage;
    inputs.turnCurrent = Amps.of(turnSim.getCurrentDrawAmps());

    inputs.hoodPosition = Radians.of(hoodSim.getAngleRads());
    inputs.hoodVelocity = RadiansPerSecond.of(hoodSim.getVelocityRadPerSec());
    inputs.hoodAppliedVoltage = hoodAppliedVoltage;
    inputs.hoodCurrent = Amps.of(hoodSim.getCurrentDrawAmps());

    inputs.flywheelVelocity = RadiansPerSecond.of(flywheelSim.getAngularVelocityRadPerSec());
    inputs.flywheelAppliedVoltage = flywheelAppliedVoltage;
    inputs.flywheelCurrent = Amps.of(flywheelSim.getCurrentDrawAmps());

    inputs.shootVelocity = RadiansPerSecond.of(shootSim.getAngularVelocityRadPerSec());
    inputs.shootAppliedVoltage = shootAppliedVoltage;
    inputs.shootCurrent = Amps.of(shootSim.getCurrentDrawAmps());
  }

  @Override
  public void setTurnVoltage(Voltage volts) {
    turnAppliedVoltage = volts;
  }

  @Override
  public void setHoodVoltage(Voltage volts) {
    hoodAppliedVoltage = volts;
  }

  @Override
  public void setFlywheelVoltage(Voltage volts) {
    flywheelAppliedVoltage = volts;
  }

  @Override
  public void setShootVoltage(Voltage volts) {
    shootAppliedVoltage = volts;
  }
}
