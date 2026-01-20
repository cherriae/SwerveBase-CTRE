package frc.robot.subsystems.Turret;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

public class TurretVisualizer {
    @Logged(name="FuelPositions")
    private ArrayList<Translation3d> fuel = new ArrayList<Translation3d>();
    @Logged(name="FuelVelocities")
    private ArrayList<Translation3d> fuelVelocity = new ArrayList<Translation3d>();
    @Logged(name="FuelTrajectory")
    private Translation3d[] trajectory = new Translation3d[50]; //last 1 second of trajectory points
    
    
    private Supplier<Pose3d> poseSupplier;
    private Supplier<ChassisSpeeds> fieldSpeedsSupplier;

    public TurretVisualizer(Supplier<Pose3d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;
    }

    private Translation3d launchVel(LinearVelocity velocity, Angle angle) {
        Pose3d robotPose = poseSupplier.get();
        ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

        double hv = Math.cos(
            angle.in(Radians) * velocity.in(MetersPerSecond)
        );

        double vv = Math.sin(
            angle.in(Radians) * velocity.in(MetersPerSecond)
        );

        double xVel = hv * Math.cos(robotPose.getRotation().toRotation2d().getRadians());
        double yVel = hv * Math.sin(robotPose.getRotation().toRotation2d().getRadians());

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        return new Translation3d(xVel, yVel, vv);
    }

    public void shootFuel(LinearVelocity velocity, Angle angle) {
        Pose3d robot = poseSupplier.get();
        Translation3d initialPos = robot.getTranslation();

        fuel.add(initialPos);
        fuelVelocity.add(launchVel(velocity, angle));
    }

    public void updateFuel(LinearVelocity velocity, Angle angle) {
        double deltaTime = 0.02; // 20 ms update time
        double gravity = 9.81; // m/s^2

        for (int i = 0; i < fuel.size(); i++) {
            Translation3d pos = fuel.get(i);
            Translation3d vel = fuelVelocity.get(i);

            double newX = pos.getX() + vel.getX() * deltaTime;
            double newY = pos.getY() + vel.getY() * deltaTime;
            double newZ = pos.getZ() + vel.getZ() * deltaTime - 0.5 * gravity * deltaTime * deltaTime;

            double newVz = vel.getZ() - gravity * deltaTime;

            fuel.set(i, new Translation3d(newX, newY, newZ));
            fuelVelocity.set(i, new Translation3d(vel.getX(), vel.getY(), newVz));

            // Remove fuel if it hits the ground
            if (newZ <= 0) {
                fuel.remove(i);
                fuelVelocity.remove(i);
                i--;
            }
        }

        Translation3d trajVelocity = launchVel(velocity, angle);
        for (int i = 0; i < trajectory.length; i++) {
            double t = i * deltaTime;
            double x = trajVelocity.getX() * t;
            double y = trajVelocity.getY() * t;
            double z = trajVelocity.getZ() * t - 0.5 * gravity * t * t;

            trajectory[i] = new Translation3d(x, y, z);
        }
    }
}