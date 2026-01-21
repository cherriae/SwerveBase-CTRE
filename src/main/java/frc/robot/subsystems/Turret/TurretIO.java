package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.*;

public interface TurretIO {
    @Logged
    public static class TurretIOInputs {
        public boolean isSim = true;

        public Angle turnPosition = Radians.zero();
        public AngularVelocity turnVelocity = RadiansPerSecond.zero();
        public Voltage turnAppliedVoltage = Volts.zero();
        public Current turnCurrent = Amps.zero();

        public Angle hoodPosition = Radians.zero();
        public AngularVelocity hoodVelocity = RadiansPerSecond.zero();
        public Voltage hoodAppliedVoltage = Volts.zero();
        public Current hoodCurrent = Amps.zero();

        public AngularVelocity flywheelVelocity = RadiansPerSecond.zero();
        public Voltage flywheelAppliedVoltage = Volts.zero();
        public Current flywheelCurrent = Amps.zero();

        public AngularVelocity shootVelocity = RadiansPerSecond.zero();
        public Voltage shootAppliedVoltage = Volts.zero();
        public Current shootCurrent = Amps.zero();
    }

    default void updateInputs(TurretIOInputs inputs) {}

    default void setTurnVoltage(Voltage volts) {}
    default void setHoodVoltage(Voltage volts) {}
    default void setFlywheelVoltage(Voltage volts) {}
    default void setShootVoltage(Voltage volts) {}
}
