package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.function.Supplier;

/** Add your docs here. */
public class TurretVisualizer {
    @Logged(name = "Fuel")
    public ArrayList<Translation3d> fuel = new ArrayList<Translation3d>();
    @Logged(name = "FuelVelocities")
    public ArrayList<Translation3d> fuelVelocities = new ArrayList<Translation3d>();
    @Logged(name = "Trajectory")
    public Translation3d[] trajectory = new Translation3d[50];
    
    private Supplier<Pose3d> poseSupplier;
    private Supplier<ChassisSpeeds> fieldSpeedsSupplier;

    public TurretVisualizer(Supplier<Pose3d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;
    }

    private Translation3d launchVel(LinearVelocity vel, Angle angle) {
        Pose3d robot = poseSupplier.get();
        ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

        double horizontalVel = Math.cos(angle.in(Radians)) * vel.in(MetersPerSecond);
        double verticalVel = Math.sin(angle.in(Radians)) * vel.in(MetersPerSecond);
        double xVel =
                horizontalVel * Math.cos(robot.getRotation().toRotation2d().getRadians());
        double yVel =
                horizontalVel * Math.sin(robot.getRotation().toRotation2d().getRadians());

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        return new Translation3d(xVel, yVel, verticalVel);
    }

    public void launchFuel(LinearVelocity vel, Angle angle) {
        Pose3d robot = poseSupplier.get();

        Translation3d initialPosition = robot.getTranslation();
        fuel.add(initialPosition);

        fuelVelocities.add(launchVel(vel, angle));
    }

    public Command repeatedlyLaunchFuel(
            Supplier<LinearVelocity> velSupplier, Supplier<Angle> angleSupplier, Turret turret) {
        return turret.runOnce(() -> launchFuel(velSupplier.get(), angleSupplier.get()))
                .andThen(Commands.waitSeconds(0.25))
                .repeatedly();
    }

    public void updateFuel(LinearVelocity vel, Angle angle) {
        double dt = 0.02; // 20 ms loop time
        double g = 9.81; // gravity in m/s^2

        for (int i = 0; i < fuel.size(); i++) {
            Translation3d position = fuel.get(i);
            Translation3d velocity = fuelVelocities.get(i);

            // Update position
            double newX = position.getX() + velocity.getX() * dt;
            double newY = position.getY() + velocity.getY() * dt;
            double newZ = position.getZ() + velocity.getZ() * dt - 0.5 * g * dt * dt;

            // Update velocity
            double newVz = velocity.getZ() - g * dt;

            fuel.set(i, new Translation3d(newX, newY, newZ));
            fuelVelocities.set(i, new Translation3d(velocity.getX(), velocity.getY(), newVz));

            // Remove fuel if it hits the ground
            if (newZ <= 0) {
                fuel.remove(i);
                fuelVelocities.remove(i);
                i--; // Adjust index after removal
            }
        }

        Translation3d trajVel = launchVel(vel, angle);
        for (int i = 0; i < trajectory.length; i++) {
            double t = i * 0.04;
            double x = trajVel.getX() * t + poseSupplier.get().getTranslation().getX();
            double y = trajVel.getY() * t + poseSupplier.get().getTranslation().getY();
            double z = trajVel.getZ() * t
                    - 0.5 * 9.81 * t * t
                    + poseSupplier.get().getTranslation().getZ();

            // Clamp z to ground
            if (z < 0) {
                z = 0;
            }
            
            trajectory[i] = new Translation3d(x, y, z);
        }
    }

    public void update3dPose(Angle azimuthAngle) {
    }
}
