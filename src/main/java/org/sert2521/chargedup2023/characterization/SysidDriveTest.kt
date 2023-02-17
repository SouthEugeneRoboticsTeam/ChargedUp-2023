package org.sert2521.chargedup2023.characterization

import com.ctre.phoenix.sensors.CANCoder
import com.kauailabs.navx.frc.AHRS
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.MotorSafety
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.sert2521.chargedup2023.ElectronicIDs
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants
import kotlin.math.PI

//This code is based on https://github.com/RegisJesuitRobotics/SysIdJavaLogger
const val dataSize = 36000

class CharacterizationModule(private val powerMotor: CANSparkMax,
                             private val angleMotor: CANSparkMax,
                             private val angleEncoder: CANCoder,
                             private val angleOffset: Double,
                             private val anglePID: PIDController,
                             private val inverted: Boolean,
                             var state: SwerveModuleState) : MotorSafety() {

    var position: SwerveModulePosition

    init {
        anglePID.enableContinuousInput(-PI * 2, PI * 2)

        powerMotor.idleMode = CANSparkMax.IdleMode.kBrake
        angleMotor.idleMode = CANSparkMax.IdleMode.kBrake

        position = SwerveModulePosition(powerMotor.encoder.position, getAngle())

        powerMotor.inverted = inverted

        powerMotor.encoder.positionConversionFactor = PhysicalConstants.powerEncoderMultiplierPosition
        powerMotor.encoder.velocityConversionFactor = PhysicalConstants.powerEncoderMultiplierVelocity
    }

    private fun getAngle(): Rotation2d {
        return if (inverted) {
            Rotation2d(-(angleEncoder.absolutePosition * PhysicalConstants.angleEncoderMultiplier - angleOffset))
        } else {
            Rotation2d(angleEncoder.absolutePosition * PhysicalConstants.angleEncoderMultiplier - angleOffset)
        }
    }

    // Should be called in periodic
    fun updateState() {
        val angle = getAngle()
        state = SwerveModuleState(powerMotor.encoder.position, angle)
        position = SwerveModulePosition(powerMotor.encoder.velocity, angle)
    }

    fun set(voltage: Double) {
        // Why isn't motor.inverted working if it isn't
        if (!inverted) {
            powerMotor.set(voltage)
        } else {
            powerMotor.set(voltage)
        }

        angleMotor.set(anglePID.calculate(state.angle.radians, 0.0))
    }

    override fun stopMotor() {
        powerMotor.stopMotor()
        angleMotor.stopMotor()
    }

    override fun getDescription(): String {
        return "Swerve Module"
    }
}

private enum class TestType {
    QUASISTATIC, DYNAMIC, OTHER
}

class DrivetrainCharacterization : TimedRobot(0.005) {
    private val imu = AHRS()

    private val data = ArrayList<Double>(dataSize)

    private var voltage = 0.0
    private var leftVoltage = 0.0
    private var rightVoltage = 0.0

    private var currentTime = 0.0
    private var startTime = 0.0

    private var testType = TestType.OTHER
    private var voltageCommand = 0.0

    private val leftModules = mutableListOf<CharacterizationModule>()
    private val rightModules = mutableListOf<CharacterizationModule>()

    init {
        LiveWindow.disableAllTelemetry()

        val modulePositions = mutableListOf<Translation2d>()
        for (moduleData in ElectronicIDs.swerveModuleData) {
            val powerMotor = CANSparkMax(moduleData.powerMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
            val angleMotor = CANSparkMax(moduleData.angleMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)

            modulePositions.add(moduleData.position)
            val module = CharacterizationModule(powerMotor, angleMotor, CANCoder(moduleData.angleEncoderID), moduleData.angleOffset, PIDController(TunedConstants.swerveAngleP, TunedConstants.swerveAngleI, TunedConstants.swerveAngleD), moduleData.inverted, SwerveModuleState())
            module.isSafetyEnabled = true

            if (moduleData.position.x == PhysicalConstants.halfSideLength) {
                rightModules.add(module)
            }

            if (moduleData.position.x == -PhysicalConstants.halfSideLength) {
                leftModules.add(module)
            }
        }
    }

    private fun update() {
        for (module in leftModules) {
            module.updateState()
        }

        for (module in rightModules) {
            module.updateState()
        }
    }

    override fun autonomousInit() {
        val testTypeString = SmartDashboard.getString("SysIdTestType", "")
        testType = when (testTypeString) {
            "Quasistatic" -> TestType.QUASISTATIC
            "Dynamic" -> TestType.DYNAMIC
            else -> TestType.OTHER
        }

        voltageCommand = SmartDashboard.getNumber("SysIdVoltageCommand", 0.0)
        startTime = Timer.getFPGATimestamp()
        data.clear()
    }

    override fun autonomousPeriodic() {
        update()

        logData(leftModules[0].position.distanceMeters, rightModules[0].position.distanceMeters, leftModules[0].state.speedMetersPerSecond, rightModules[0].state.speedMetersPerSecond, Math.toRadians(imu.angle), Math.toRadians(imu.rate))

        for (module in leftModules) {
            module.set(voltage)
        }

        for (module in rightModules) {
            module.set(voltage)
        }
    }

    override fun teleopPeriodic() {
        pushNTDiagnostics()
    }

    override fun disabledInit() {
        for (module in leftModules) {
            module.set(0.0)
        }

        for (module in rightModules) {
            module.set(0.0)
        }

        sendData()
    }

    override fun disabledPeriodic() {
        pushNTDiagnostics()
    }

    override fun testPeriodic() {
        pushNTDiagnostics()
    }

    private fun updateData() {
        currentTime = Timer.getFPGATimestamp()
        voltage = when (testType) {
            TestType.QUASISTATIC -> voltageCommand * (currentTime - startTime)
            TestType.DYNAMIC -> voltageCommand
            TestType.OTHER -> 0.0
        }
    }

    private fun logData(leftPosition: Double, rightPosition: Double, leftRate: Double, rightRate: Double, gyroPosition: Double, gyroRate: Double) {
        updateData()
        if (data.size < dataSize) {
            data.add(currentTime)
            data.add(leftVoltage)
            data.add(rightVoltage)
            data.add(leftPosition)
            data.add(rightPosition)
            data.add(leftRate)
            data.add(rightRate)
            data.add(gyroPosition)
            data.add(gyroRate)
        }

        leftVoltage = voltage
        rightVoltage = voltage
    }

    private fun sendData() {
        SmartDashboard.putBoolean("SysIdOverflow", data.size > dataSize)
        val dataString = data.toString()
        SmartDashboard.putString("SysIdTelemetry", data.toString().substring(1, dataString.length - 1))
        reset()
    }

    private fun reset() {
        voltage = 0.0
        currentTime = 0.0
        startTime = 0.0
        leftVoltage = 0.0
        rightVoltage = 0.0
        data.clear()
    }

    private fun pushNTDiagnostics() {
        update()
        SmartDashboard.putNumber("Left Position", leftModules[0].position.distanceMeters)
        SmartDashboard.putNumber("Right Position", rightModules[0].position.distanceMeters)
        SmartDashboard.putNumber("Left Velocity", leftModules[0].state.speedMetersPerSecond)
        SmartDashboard.putNumber("Right Velocity", rightModules[0].state.speedMetersPerSecond)
        SmartDashboard.putNumber("Gyro Reading", Math.toRadians(imu.angle))
        SmartDashboard.putNumber("Gyro Rate", Math.toRadians(imu.rate))
    }
}