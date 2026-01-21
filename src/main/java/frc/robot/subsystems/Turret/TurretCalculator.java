package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.TurretConstants.ROBOT_TO_TURRET_TRANSFORM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class TurretCalculator {
    public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        return Meters.of(robot.getTranslation().getDistance(target.toTranslation2d()));
    }

    // see https://www.desmos.com/geometry/l4edywkmha
    public static Angle calculateAngleFromVelocity(Pose2d robot, LinearVelocity velocity, Translation3d target) {
        double g = MetersPerSecondPerSecond.of(9.81).in(InchesPerSecondPerSecond);
        double vel = velocity.in(InchesPerSecond);
        double x_dist = getDistanceToTarget(robot, target).in(Inches);
        double y_dist = target.getMeasureZ()
                .minus(ROBOT_TO_TURRET_TRANSFORM.getMeasureZ())
                .in(Inches);
        double angle = Math.atan(
                ((vel * vel) - Math.sqrt(Math.max(0, Math.pow(vel, 4) - g * (g * x_dist * x_dist + 2 * y_dist * vel * vel))))
                        / (g * x_dist));
        return Radians.of(angle);
    }

    // calculates how long it will take for a projectile to travel a set distance given its initial velocity and angle
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

    // calculates the angle of a turret relative to the robot to hit a target
    public static Angle calculateAzimuthAngle(Pose2d robot, Translation3d target) {
        Translation2d turretTranslation = new Pose3d(robot)
                .transformBy(ROBOT_TO_TURRET_TRANSFORM)
                .toPose2d()
                .getTranslation();

        Translation2d direction = target.toTranslation2d().minus(turretTranslation);

        return Radians.of(MathUtil.inputModulus(
                direction.getAngle().minus(robot.getRotation()).getRadians() + Math.PI, -Math.PI, Math.PI));
    }

    // Move a target a set time in the future along a velocity defined by fieldSpeeds
    public static Translation3d predictTargetPos(Translation3d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
        double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds);
        double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);

        return new Translation3d(predictedX, predictedY, target.getZ());
    }

    // use an iterative lookahead approach to determine shot parameters for a moving robot
    public static ShotData iterativeMovingShot(
            Pose2d robot,
            ChassisSpeeds fieldSpeeds,
            Translation3d target,
            int iterations) {
        // Initial estimation assuming stationary target
        LinearVelocity exitVelocity = MetersPerSecond.of(15);
        Angle hoodAngle = calculateAngleFromVelocity(robot, exitVelocity, target);
        ShotData shot = new ShotData(exitVelocity, hoodAngle, target);

        Distance distance = getDistanceToTarget(robot, target);
        Time timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(), distance);
        Translation3d predictedTarget = target;

        for (int i = 0; i < iterations; i++) {
            // Predict where the target will be after current time-of-flight
            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            // Recompute shot parameters to the predicted target
            hoodAngle = calculateAngleFromVelocity(robot, exitVelocity, predictedTarget);
            shot = new ShotData(exitVelocity, hoodAngle, predictedTarget);
            timeOfFlight = calculateTimeOfFlight(
                    shot.getExitVelocity(), shot.getHoodAngle(), getDistanceToTarget(robot, predictedTarget));
        }

        return shot;
    }

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
}
