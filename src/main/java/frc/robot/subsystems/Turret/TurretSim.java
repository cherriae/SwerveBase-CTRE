package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretSim {
    // sim
    private final DCMotor _turnMotor = DCMotor.getKrakenX44Foc(1);
    private final SingleJointedArmSim turnSim = new SingleJointedArmSim(
        _turnMotor,
        2, 
        0.001, 
        0.1,
        0, 
        2 * Math.PI,
        false, 
        0, 
        0.0, 
        0.0 
    );

    private final DCMotor _hoodMotor = DCMotor.getKrakenX44Foc(1);
    private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(
        _hoodMotor,
        2,
        0.005,
        0.1,
        0,
        Math.PI,
        false,
        0,
        0.002,
        0.0002
    );

    private final DCMotor _flywheelMotor = DCMotor.getKrakenX60Foc(1);
    private final FlywheelSim flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(_flywheelMotor, 0.005, 2), 
        _flywheelMotor, 0.005
    );
    
    private final DCMotor _shootMotor = DCMotor.getKrakenX60Foc(1);
    private final FlywheelSim shootSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(_shootMotor, 0.005, 2), 
        _shootMotor, 0.005
    );

    // input values
    @Logged(name="TurnCurrent")
    public Current turnCurrent = Amps.zero();

    @Logged(name="TurnPosition")
    public Angle turnPosition = Radians.zero();

    @Logged(name="TurnVelocity")
    public AngularVelocity turnVelocity = RadiansPerSecond.zero();

    @Logged(name="HoodPosition")
    public Angle hoodPosition = Radians.zero();

    @Logged(name="HoodVelocity")
    public AngularVelocity hoodVelocity = RadiansPerSecond.zero();

    @Logged(name="HoodCurrent")
    public Current hoodCurrent = Amps.zero();

    @Logged(name="FlywheelCurrent")
    public Current flywheelCurrent = Amps.zero();

    @Logged(name="FlywheelVelocity")
    public AngularVelocity flywheelVelocity = RadiansPerSecond.zero();
    
    @Logged(name="ShootCurrent")
    public Current shootCurrent = Amps.zero();
    
    @Logged(name="ShootVelocity")
    public AngularVelocity shootVelocity = RadiansPerSecond.zero();

    public TurretSim() {
        setTurnOutput(Volts.zero());
        setHoodOutput(Volts.zero());
        setFlywheelOutput(Volts.zero());
        setShootOutput(Volts.zero());
    }

    public void updateInputs() {
        turnSim.update(0.02);
        hoodSim.update(0.02);
        flywheelSim.update(0.02);
        shootSim.update(0.02);

        turnPosition = Radians.of(turnSim.getAngleRads());
        turnVelocity = RadiansPerSecond.of(turnSim.getVelocityRadPerSec());
        turnCurrent = Amps.of(turnSim.getCurrentDrawAmps());

        hoodPosition = Radians.of(hoodSim.getAngleRads());
        hoodVelocity = RadiansPerSecond.of(hoodSim.getVelocityRadPerSec());
        hoodCurrent = Amps.of(hoodSim.getCurrentDrawAmps());
        
        flywheelVelocity = RadiansPerSecond.of(flywheelSim.getAngularVelocityRadPerSec());
        flywheelCurrent = Amps.of(flywheelSim.getCurrentDrawAmps());
        
        shootVelocity = RadiansPerSecond.of(shootSim.getAngularVelocityRadPerSec());
        shootCurrent = Amps.of(shootSim.getCurrentDrawAmps());
    }

    public void setTurnOutput(Voltage voltage) {
        turnSim.setInput(voltage.in(Volts));
    }

    public void setHoodOutput(Voltage voltage) {
        hoodSim.setInput(voltage.in(Volts));
    }

    public void setFlywheelOutput(Voltage voltage) {
        flywheelSim.setInput(voltage.in(Volts));
    }

    public void setShootOutput(Voltage voltage) {
        shootSim.setInput(voltage.in(Volts));
    }

    @Logged(name="TurretPose")
    public Pose3d update3dPose(Angle azimuthAngle) {
        return new Pose3d(0, 0, 0,
            new Rotation3d(0, 0, azimuthAngle.in(Radians))
        );
    }
}