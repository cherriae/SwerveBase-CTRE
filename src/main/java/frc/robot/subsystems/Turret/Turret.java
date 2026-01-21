package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
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

import java.util.function.Supplier;

public class Turret extends AdvancedSubsystem {
  @Logged
  private final TurretVisualizer turretVisualizer;
  private final TurretIO io;
  @Logged
  private final TurretIO.TurretIOInputs inputs = new TurretIO.TurretIOInputs();
  
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  @Logged(name = "AzimuthAngle")
  public Angle azimuthAngle = Radians.zero();

  private ShotData calculatedShot;

  public static final Distance FIELD_LENGTH = Inches.of(650.12);
  public static final Distance FIELD_WIDTH = Inches.of(316.64);
  public static final Translation3d HUB_BLUE =
      new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2), Inches.of(56.4));
  public static final Translation3d HUB_RED =
      new Translation3d(FIELD_LENGTH.minus(Inches.of(181.56)), FIELD_WIDTH.div(2), Inches.of(56.4));

  @Logged(name = "CurrentTarget")
  Translation3d currentTarget =
      DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? HUB_BLUE : HUB_RED;

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
    return Meters.of(
        robot.getTranslation().getDistance(new Translation2d(target.getX(), target.getY())));
  }

  public static Angle calculateAngleFromVelocity(
      Pose2d robot, LinearVelocity velocity, Translation3d target) {
    double g = MetersPerSecondPerSecond.of(9.81).in(MetersPerSecondPerSecond);
    double vel = velocity.in(MetersPerSecond);
    double x_dist = getDistanceToTarget(robot, target).in(Meters);
    double y_dist = target.getZ() - Constants.TurretConstants.turretOffset.getZ();

    double angle =
        Math.atan(
            ((vel * vel)
                    + Math.sqrt(
                        Math.max(
                            0,
                            Math.pow(vel, 4) - g * (g * x_dist * x_dist + 2 * y_dist * vel * vel))))
                / (g * x_dist));
    return Radians.of(angle);
  }

  // Calculates how long it will take for a projectile to travel a set distance given its initial
  // velocity and angle
  public static Time calculateTimeOfFlight(
      LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
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
  public static Translation3d predictTargetPos(
      Translation3d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
    double t = timeOfFlight.in(Seconds);
    double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * t;
    double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * t;
    return new Translation3d(predictedX, predictedY, target.getZ());
  }

  public static Angle calculateAzimuthAngle(Pose2d robot, Translation3d target) {
    return TurretCalculator.calculateAzimuthAngle(robot, target);
  }

  // Calculate shot parameters based on distance and height difference
  // Uses your Desmos math for trajectory
  public static ShotData calculateShotData(Pose2d robot, Translation3d target) {
    // Base velocity calculation - can be tuned based on distance
    double baseVelocity = Constants.TurretConstants.BASE_VELOCITY.in(MetersPerSecond);
    LinearVelocity exitVelocity = MetersPerSecond.of(baseVelocity);

    // Calculate hood angle using trajectory
    Angle hoodAngle = TurretCalculator.calculateAngleFromVelocity(robot, exitVelocity, target);

    return new ShotData(exitVelocity, hoodAngle, target);
  }

  // Iteratively refine a moving shot using your ballistic math
  public static ShotData iterativeMovingShotFromFunnelClearance(
      Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
    TurretCalculator.ShotData data = TurretCalculator.iterativeMovingShot(
      robot, fieldSpeeds, target, iterations);
    return new ShotData(
      data.getExitVelocity(), data.getHoodAngle(), data.getTarget()
    );
  }

  public Turret(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    this.poseSupplier = poseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    io = new TurretIOSim();

    turretVisualizer =
        new TurretVisualizer(
            // Simulating turret pose relative to robot
            () -> {
              Pose2d robotPose = poseSupplier.get();
              Translation3d offset = Constants.TurretConstants.turretOffset;
              Translation2d offset2d =
                  new Translation2d(offset.getX(), offset.getY())
                      .rotateBy(robotPose.getRotation());

              return new Pose3d(
                  robotPose.getX() + offset2d.getX(),
                  robotPose.getY() + offset2d.getY(),
                  offset.getZ(),
                  new Rotation3d(0, 0, robotPose.getRotation().getRadians() + inputs.turnPosition.in(Radians)));
            },
            chassisSpeedsSupplier
            );

    SmartDashboard.putData(
        "Launch Fuel",
        this.runOnce(
            () -> {
              // Use shooter radius for conversion
              turretVisualizer.launchFuel(
                  MetersPerSecond.of(25), // Fast shot for testing
                  Radians.of(Math.toRadians(45)));
            }));
    
    // Default command: Spin flywheel and periodically launch to demonstrate arc
    this.setDefaultCommand(
        this.run(() -> {
            io.setFlywheelVoltage(Volts.of(10)); // Spin flywheel
            io.setHoodVoltage(Volts.of(0)); 

            // Simple timer for periodic shooting in sim
            double time = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            if (time % 0.1 < 0.02) {
                 turretVisualizer.launchFuel(
                    angularToLinearVelocity(inputs.flywheelVelocity, Constants.TurretConstants.flywheelRadius),
                    inputs.hoodPosition
                 );
            }
        })
    );
  }

  @Override
  public void periodic() {
    // Ported from FRC 5000 2026 Codebase
    io.updateInputs(inputs);

    currentTarget =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? HUB_BLUE : HUB_RED;

    Pose2d robot = poseSupplier.get();
    ChassisSpeeds fieldSpeeds = chassisSpeedsSupplier.get();

    calculatedShot = iterativeMovingShotFromFunnelClearance(
            robot, fieldSpeeds, currentTarget, 2);
    
    // Calculates azimuth angle to target
    azimuthAngle = TurretCalculator.calculateAzimuthAngle(robot, calculatedShot.getTarget());

    // Update Visualizer
    turretVisualizer.updateFuel(
            angularToLinearVelocity(inputs.flywheelVelocity, Constants.TurretConstants.flywheelRadius), 
            inputs.hoodPosition);
    
    turretVisualizer.update3dPose(inputs.turnPosition);
  }

  @Override
  public void simulationPeriodic() {
    // Simple P-Control for Physics Sim Visualization
    // TURRET
    double currentTurn = inputs.turnPosition.in(Radians);
    double targetTurn = azimuthAngle.in(Radians);
    
    // Calculate shortest path error within [-PI, PI]
    double turnError = edu.wpi.first.math.MathUtil.angleModulus(targetTurn - currentTurn);
    
    io.setTurnVoltage(Volts.of(turnError * 20.0)); // kP = 20

    // HOOD
    double hoodError = calculatedShot.getHoodAngle().in(Radians) - inputs.hoodPosition.in(Radians);
    io.setHoodVoltage(Volts.of(hoodError * 20.0)); // kP = 20

    // SHOOTER/FLYWHEEL
    AngularVelocity targetFlywheelVel = linearToAngularVelocity(
      calculatedShot.getExitVelocity(), Constants.TurretConstants.flywheelRadius);
    double flywheelError = targetFlywheelVel.in(RadiansPerSecond) - inputs.flywheelVelocity.in(RadiansPerSecond);
    io.setFlywheelVoltage(Volts.of(flywheelError * 0.1)); // kP = 0.1 (velocity control need less kP usually or feedforward)
    // Add feedforward for better sim response (Sim behaves like perfect dc motor)
    io.setFlywheelVoltage(Volts.of(flywheelError * 0.1 + (targetFlywheelVel.in(RadiansPerSecond) * 0.02))); 

    // Shoot (using same logic as flywheel for now since we don't have separate mechanism in IO yet fully separated)
    io.setShootVoltage(Volts.of(flywheelError * 0.1));
  }
}
