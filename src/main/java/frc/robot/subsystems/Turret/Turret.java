package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.AdvancedSubsystem;
import frc.robot.Constants;

public class Turret extends AdvancedSubsystem {
    private final TurretVisualizer turretVisualizer;
    private final TurretSim turretSim = new TurretSim();

    public static final Distance FIELD_LENGTH = Inches.of(650.12);
    public static final Distance FIELD_WIDTH = Inches.of(316.64);
    public static final Translation3d HUB_BLUE =
            new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2), Inches.of(56.4));
    public static final Translation3d HUB_RED =
            new Translation3d(FIELD_LENGTH.minus(Inches.of(181.56)), FIELD_WIDTH.div(2), Inches.of(56.4));

    @Logged(name="CurrentTarget")
    Translation3d currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? HUB_BLUE
            : HUB_RED;

    // Shot data record
    public record ShotData(double exitVelocity, double hoodAngle, Translation3d target) {
        public ShotData(LinearVelocity exitVelocity, Angle hoodAngle, Translation3d target) {
            this(exitVelocity.in(MetersPerSecond), hoodAngle.in(Radians), target);
        }

        public LinearVelocity getExitVelocity() {
            return MetersPerSecond.of(this.exitVelocity);
        }

        public Angle getHoodAngle() {
            return Radians.of(this.hoodAngle);
        }

        public Translation3d getTarget() {
            return this.target;
        }
    }

    public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        return Meters.of(robot.getTranslation().getDistance(new Translation2d(target.getX(), target.getY())));
    }


    public static Angle calculateAngleFromVelocity(Pose2d robot, LinearVelocity velocity, Translation3d target) {
        double g = MetersPerSecondPerSecond.of(9.81).in(MetersPerSecondPerSecond);
        double vel = velocity.in(MetersPerSecond);
        double x_dist = getDistanceToTarget(robot, target).in(Meters);
        double y_dist = target.getZ() - Constants.TurretConstants.turretOffset.getZ();
        
        double angle = Math.atan(
            ((vel * vel) + Math.sqrt(
                Math.max(0, Math.pow(vel, 4) - g * (g * x_dist * x_dist + 2 * y_dist * vel * vel))
            )) / (g * x_dist)
        );
        return Radians.of(angle);
    }

    // Calculates how long it will take for a projectile to travel a set distance given its initial velocity and angle
    public static Time calculateTimeOfFlight(LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
        double vel = exitVelocity.in(MetersPerSecond);
        double angle = hoodAngle.in(Radians);
        double dist = distance.in(Meters);
        return Seconds.of(dist / (vel * Math.cos(angle)));
    }

    public static AngularVelocity linearToAngularVelocity(LinearVelocity vel, Distance radius) {
        return RadiansPerSecond.of(vel.in(MetersPerSecond) / radius.in(Meters));
    }

    public static LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
        return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters));
    }

    // Predict target position after time-of-flight given current field-relative speeds
    public static Translation3d predictTargetPos(Translation3d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
        double t = timeOfFlight.in(Seconds);
        double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * t;
        double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * t;
        return new Translation3d(predictedX, predictedY, target.getZ());
    }

    // Calculates the angle of a turret relative to the robot to hit a target
    public static Angle calculateAzimuthAngle(Pose2d robot, Translation3d target) {
        Translation2d turretTranslation = robot.getTranslation()
            .plus(new Translation2d(Constants.TurretConstants.turretOffset.getX(), 
                                    Constants.TurretConstants.turretOffset.getY()));

        Translation2d direction = new Translation2d(target.getX(), target.getY()).minus(turretTranslation);

        return Radians.of(MathUtil.inputModulus(
            direction.getAngle().minus(robot.getRotation()).getRadians(), 0, 2 * Math.PI));
    }

    // Calculate shot parameters based on distance and height difference
    // Uses your Desmos math for trajectory
    public static ShotData calculateShotData(Pose2d robot, Translation3d target) {
        // Base velocity calculation - can be tuned based on distance
        double baseVelocity = Constants.TurretConstants.BASE_VELOCITY.in(MetersPerSecond);
        LinearVelocity exitVelocity = MetersPerSecond.of(baseVelocity);
        
        // Calculate hood angle using trajectory
        Angle hoodAngle = calculateAngleFromVelocity(robot, exitVelocity, target);
        
        return new ShotData(exitVelocity, hoodAngle, target);
    }

    // Iteratively refine a moving shot using your ballistic math
    public static ShotData iterativeMovingShotFromFunnelClearance(
            Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
        // Initial estimation assuming stationary target
        ShotData shot = calculateShotData(robot, target);
        Distance distance = getDistanceToTarget(robot, target);
        Time timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(), distance);
        Translation3d predictedTarget = target;

        for (int i = 0; i < iterations; i++) {
            // Predict where the target will be after current time-of-flight
            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            // Recompute shot parameters to the predicted target
            shot = calculateShotData(robot, predictedTarget);
            timeOfFlight = calculateTimeOfFlight(
                shot.getExitVelocity(), shot.getHoodAngle(), getDistanceToTarget(robot, predictedTarget));
        }

        return shot;
    }

    public Turret() {
        turretVisualizer = new TurretVisualizer(
                // Simulating turret pose relative to robot
                () -> {
                    Pose2d robotPose = new Pose2d();
                    Translation3d offset = Constants.TurretConstants.turretOffset;
                    return new Pose3d(
                        robotPose.getX() + offset.getX(),
                        robotPose.getY() + offset.getY(),
                        offset.getZ(),
                        new Rotation3d(0, 0, robotPose.getRotation().getRadians())
                    );
                },
            null
        );

        SmartDashboard.putData(
            "Launch Fuel",
            this.runOnce( () -> {
                // Use shooter radius for conversion
                angularToLinearVelocity(
                    RadiansPerSecond.of(1),
                    Constants.TurretConstants.shooterRadius
                );
            })
        );
    }

    @Override
    public void simulationPeriodic() {
        turretSim.updateInputs();
        
        currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? HUB_BLUE
            : HUB_RED;
        
        // Update visualizer with current shooter state
        turretVisualizer.updateFuel(
             angularToLinearVelocity(turretSim.flywheelVelocity, Constants.TurretConstants.flywheelRadius), 
             turretSim.hoodPosition);
    }
}
