package org.sert2521.chargedup2023.subsystems

import com.ctre.phoenix.sensors.CANCoder
import com.kauailabs.navx.frc.AHRS
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.MotorSafety
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.sert2521.chargedup2023.*
import kotlin.math.*

class SwerveModule(private val powerMotor: CANSparkMax,
                   private val powerFeedforward: SimpleMotorFeedforward,
                   private val powerPID: PIDController,
                   private val angleMotor: CANSparkMax,
                   private val angleEncoder: CANCoder,
                   private val angleOffset: Double,
                   private val anglePID: PIDController,
                   private val centerRotation: Rotation2d,
                   private val inverted: Boolean,
                   var state: SwerveModuleState,
                   shouldOptimize: Boolean,
                   brakeMode: Boolean) : MotorSafety() {
    var doesOptimize = shouldOptimize
        private set

    var position: SwerveModulePosition

    init {
        if (doesOptimize) {
            anglePID.enableContinuousInput(-PI, PI)
        } else {
            anglePID.enableContinuousInput(-PI * 2, PI * 2)
        }

        setMotorMode(!brakeMode)

        powerMotor.inverted = inverted

        powerMotor.encoder.positionConversionFactor = PhysicalConstants.powerEncoderMultiplierPosition
        powerMotor.encoder.velocityConversionFactor = PhysicalConstants.powerEncoderMultiplierVelocity

        position = SwerveModulePosition(powerMotor.encoder.position, getAngle())
    }

    private fun getAngle(): Rotation2d {
        return if (inverted) {
            Rotation2d(-(angleEncoder.absolutePosition * PhysicalConstants.angleEncoderMultiplier - angleOffset))
        } else {
            Rotation2d(angleEncoder.absolutePosition * PhysicalConstants.angleEncoderMultiplier - angleOffset)
        }
    }

    fun setOptimize(value: Boolean) {
        doesOptimize = value

        // Should these be halved
        if (doesOptimize) {
            anglePID.enableContinuousInput(-PI, PI)
        } else {
            anglePID.enableContinuousInput(-PI * 2, PI * 2)
        }
    }

    // Should be called in periodic
    fun updateState() {
        val angle = getAngle()
        state = SwerveModuleState(powerMotor.encoder.velocity, angle)
        position = SwerveModulePosition(powerMotor.encoder.position, angle)
    }

    fun set(wanted: SwerveModuleState) {
        // Using state because it should be updated and getVelocity and getAngle (probably) spend time over CAN
        val optimized = if (doesOptimize) {
            SwerveModuleState.optimize(wanted, state.angle)
        } else {
            wanted
        }

        val feedforward = powerFeedforward.calculate(optimized.speedMetersPerSecond)
        val pid = if (inverted) {
            powerPID.calculate(-state.speedMetersPerSecond, optimized.speedMetersPerSecond)
        } else {
            powerPID.calculate(state.speedMetersPerSecond, optimized.speedMetersPerSecond)
        }

        // Why isn't motor.inverted working if it isn't
        if (!inverted) {
            powerMotor.set((feedforward + pid) / 12.0)
        } else {
            powerMotor.set(-(feedforward + pid) / 12.0)
        }
        angleMotor.set(anglePID.calculate(state.angle.radians, optimized.angle.radians))
    }

    fun enterBrakePos() {
        set(SwerveModuleState(0.0, centerRotation))
    }

    fun setMotorMode(coast: Boolean) {
        if (coast) {
            powerMotor.idleMode = CANSparkMax.IdleMode.kCoast
            angleMotor.idleMode = CANSparkMax.IdleMode.kCoast
        } else {
            powerMotor.idleMode = CANSparkMax.IdleMode.kBrake
            angleMotor.idleMode = CANSparkMax.IdleMode.kBrake
        }
    }

    override fun stopMotor() {
        powerMotor.stopMotor()
        angleMotor.stopMotor()
    }

    override fun getDescription(): String {
        return "Swerve Module"
    }
}

object Drivetrain : SubsystemBase() {
    private val imu = AHRS()

    private val cams: Array<PhotonCamera>
    private val photonPoseEstimators: Array<PhotonPoseEstimator>

    private val kinematics: SwerveDriveKinematics
    private var modules: Array<SwerveModule>
    private val odometry: SwerveDriveOdometry
    private val poseEstimator: SwerveDrivePoseEstimator

    private var pose = Pose2d()
    private var visionPose = Pose2d()

    private var prevPose = Pose2d()
    private var prevTime = Timer.getFPGATimestamp()

    var deltaPose = Pose2d()
        private set

    // False is broken
    var doesOptimize = ConfigConstants.drivetrainOptimized
        private set

    init {
        val modulePositions = mutableListOf<Translation2d>()
        val modulesList = mutableListOf<SwerveModule>()

        // Maybe the module should create the motors
        for (moduleData in ElectronicIDs.swerveModuleData) {
            val powerMotor = CANSparkMax(moduleData.powerMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
            val angleMotor = CANSparkMax(moduleData.angleMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)

            modulePositions.add(moduleData.position)
            val module = createModule(powerMotor, angleMotor, moduleData)
            module.isSafetyEnabled = true
            modulesList.add(module)
        }

        modules = modulesList.toTypedArray()

        val positions = mutableListOf<SwerveModulePosition>()

        for (module in modules) {
            module.updateState()
            positions.add(module.position)
        }

        val positionsArray = positions.toTypedArray()

        kinematics = SwerveDriveKinematics(*modulePositions.toTypedArray())
        odometry = SwerveDriveOdometry(kinematics, -imu.rotation2d, positionsArray, Pose2d())
        poseEstimator = SwerveDrivePoseEstimator(kinematics, -imu.rotation2d, positionsArray, Pose2d(), TunedConstants.encoderDeviations, TunedConstants.defaultVisionDeviations)

        val camsList = mutableListOf<PhotonCamera>()
        val photonPoseEstimatorsList = mutableListOf<PhotonPoseEstimator>()
        for (camData in ElectronicIDs.camData) {
            val cam = PhotonCamera(camData.first)
            camsList.add(cam)
            // Field gets updated before run so it can be null
            photonPoseEstimatorsList.add(PhotonPoseEstimator(PhysicalConstants.field, PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS, cam, camData.second))
        }

        cams = camsList.toTypedArray()
        photonPoseEstimators = photonPoseEstimatorsList.toTypedArray()
    }

    // Fix this nonsense
    fun getPose(): Pose2d {
        return Pose2d(pose.y, pose.x, -pose.rotation)
    }

    // Fix this nonsense
    fun getVisionPose(): Pose2d {
        return Pose2d(visionPose.y, visionPose.x, -visionPose.rotation)
    }

    private fun createModule(powerMotor: CANSparkMax, angleMotor: CANSparkMax, moduleData: SwerveModuleData): SwerveModule {
        return SwerveModule(powerMotor,
            SimpleMotorFeedforward(TunedConstants.swervePowerS, TunedConstants.swervePowerV, TunedConstants.swervePowerA),
            PIDController(TunedConstants.swervePowerP, TunedConstants.swervePowerI, TunedConstants.swervePowerD),
            angleMotor,
            CANCoder(moduleData.angleEncoderID),
            moduleData.angleOffset,
            PIDController(TunedConstants.swerveAngleP, TunedConstants.swerveAngleI, TunedConstants.swerveAngleD),
            Rotation2d(atan2(moduleData.position.y, moduleData.position.x)),
            moduleData.inverted,
            SwerveModuleState(),
            doesOptimize,
            true
        )
    }

    override fun periodic() {
        val positions = mutableListOf<SwerveModulePosition>()

        for (module in modules) {
            module.updateState()
            positions.add(module.position)
        }

        val positionsArray = positions.toTypedArray()

        pose = odometry.update(-imu.rotation2d, positionsArray)
        visionPose = poseEstimator.update(-imu.rotation2d, positionsArray)

        for (photonPoseEstimator in photonPoseEstimators) {
            val poseOutput = photonPoseEstimator.update()
            if (poseOutput.isPresent) {
                val currVisionPoseData = poseOutput.get()
                val currVisionPose = currVisionPoseData.estimatedPose.toPose2d()

                val color = Input.getColor()
                if (color == DriverStation.Alliance.Blue) {
                    poseEstimator.addVisionMeasurement(Pose2d(currVisionPose.y, currVisionPose.x, -currVisionPose.rotation), currVisionPoseData.timestampSeconds)
                } else if (color == DriverStation.Alliance.Red) {
                    poseEstimator.addVisionMeasurement(Pose2d(currVisionPose.y, PhysicalConstants.fieldLength - currVisionPose.x, Rotation2d(PI) - currVisionPose.rotation), currVisionPoseData.timestampSeconds)
                }
            }
        }

        val currTime = Timer.getFPGATimestamp()
        val deltaTime = currTime - prevTime

        deltaPose = Pose2d((pose.y - prevPose.y) / deltaTime, (pose.x - prevPose.x) / deltaTime, -(pose.rotation - prevPose.rotation) / deltaTime)

        prevPose = pose
        prevTime = currTime
    }

    fun setOptimize(value: Boolean) {
        doesOptimize = value

        for (module in modules) {
            module.setOptimize(doesOptimize)
        }
    }

    fun setNewPose(newPose: Pose2d) {
        pose = Pose2d(newPose.y, newPose.x, -newPose.rotation)

        val positions = mutableListOf<SwerveModulePosition>()

        for (module in modules) {
            module.updateState()
            positions.add(module.position)
        }

        val positionsArray = positions.toTypedArray()

        odometry.resetPosition(-imu.rotation2d, positionsArray, pose)
    }

    fun setNewVisionPose(newPose: Pose2d) {
        visionPose = Pose2d(newPose.y, newPose.x, -newPose.rotation)

        val positions = mutableListOf<SwerveModulePosition>()

        for (module in modules) {
            module.updateState()
            positions.add(module.position)
        }

        val positionsArray = positions.toTypedArray()

        poseEstimator.resetPosition(-imu.rotation2d, positionsArray, visionPose)
    }

    fun setVisionStandardDeviations() {
        poseEstimator.setVisionMeasurementStdDevs(TunedConstants.defaultVisionDeviations)
    }

    fun setVisionAlignDeviations() {
        poseEstimator.setVisionMeasurementStdDevs(TunedConstants.alignVisionDeviations)
    }

    fun camerasConnected(): Boolean {
        for (cam in cams) {
            if (!cam.isConnected) {
                return false
            }
        }

        return true
    }

    fun getAccelSqr(): Double {
        return (imu.worldLinearAccelY.pow(2) + imu.worldLinearAccelX.pow(2)).toDouble()
    }

    private fun feed() {
        for (module in modules) {
            module.feed()
        }
    }

    fun drive(chassisSpeeds: ChassisSpeeds) {
        // Maybe desaturate wheel speeds
        // Fix this ChassisSpeeds nonsense
        val wantedStates = kinematics.toSwerveModuleStates(ChassisSpeeds(-chassisSpeeds.vyMetersPerSecond, -chassisSpeeds.vxMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond))

        for (i in wantedStates.indices) {
            modules[i].set(wantedStates[i])
        }

        feed()
    }

    fun getTiltDirection(): Translation2d {
        val unNormalized = Translation2d(atan(Units.degreesToRadians(imu.roll.toDouble())), atan(Units.degreesToRadians(imu.pitch.toDouble())))
        val norm = unNormalized.norm

        if (norm == 0.0) {
            return unNormalized
        }

        return unNormalized / norm
    }

    fun getTilt(): Double {
        return atan(sqrt(tan(Units.degreesToRadians(imu.pitch.toDouble())).pow(2) + tan(Units.degreesToRadians(imu.roll.toDouble())).pow(2)))
    }

    fun enterBrakePos() {
        for (module in modules) {
            module.enterBrakePos()
        }

        feed()
    }

    fun setMode(coast: Boolean) {
        for (module in modules) {
            module.setMotorMode(coast)
        }
    }

    fun stop() {
        for (module in modules) {
            module.stopMotor()
        }

        feed()
    }
}