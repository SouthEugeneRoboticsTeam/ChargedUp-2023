package org.sert2521.chargedup2023.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.chargedup2023.ConfigConstants
import org.sert2521.chargedup2023.ElectronicIDs
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.commands.SetElevator
import kotlin.math.PI

object Elevator : SubsystemBase() {
    private val extendMotorOne = CANSparkMax(ElectronicIDs.elevatorMotorOne, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val extendMotorTwo = CANSparkMax(ElectronicIDs.elevatorMotorTwo, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val extendEncoder = extendMotorOne.encoder

    private val upperExtension = DigitalInput(ElectronicIDs.elevatorUpperExtension)
    private val lowerExtension = DigitalInput(ElectronicIDs.elevatorLowerExtension)

    var extensionInited = false
        private set

    var angleInited = false
        private set

    private val angleMotor = CANSparkMax(ElectronicIDs.elevatorAngleMotor, CANSparkMaxLowLevel.MotorType.kBrushless)
    // Maybe add filter?
    private val trueAngleEncoder = DutyCycleEncoder(ElectronicIDs.elevatorEncoder)

    init {
        extendMotorOne.idleMode = CANSparkMax.IdleMode.kBrake
        extendMotorTwo.idleMode = CANSparkMax.IdleMode.kBrake
        angleMotor.idleMode = CANSparkMax.IdleMode.kBrake

        extendMotorTwo.follow(extendMotorOne, true)

        extendEncoder.positionConversionFactor = PhysicalConstants.elevatorExtensionConversion

        trueAngleEncoder.distancePerRotation = PhysicalConstants.elevatorAngleConversion

        // Check this
        val holdCommand = InstantCommand({ SetElevator(extensionMeasure(), angleMeasure(), false) })
        holdCommand.addRequirements(this)
        defaultCommand = holdCommand
    }

    override fun periodic() {
        val atTopExtension = extensionAtTop()
        val atBottomExtension = extensionAtBottom()
        val safe = extensionSafe()

        val atTopAngle = angleAtTop()

        if (atTopExtension) {
            extendEncoder.position = PhysicalConstants.elevatorExtensionTop
            extensionInited = true
        }

        if (atBottomExtension) {
            extendEncoder.position = PhysicalConstants.elevatorExtensionBottom
            extensionInited = true
        }

        if (!extensionInited && safe) {
            extendMotorOne.setVoltage(ConfigConstants.extensionResetVoltage)
        }

        if (atTopAngle) {
            angleInited = true
        }

        if (!angleInited) {
            extendMotorOne.setVoltage(ConfigConstants.angleResetVoltage)
        }

        if (!safe || (extendMotorOne.appliedOutput > 0 && atTopExtension) || (extendMotorOne.appliedOutput < 0 && atBottomExtension)) {
            extendMotorOne.stopMotor()
        }

        if ((angleMotor.appliedOutput > 0 && angleAtTop()) || (angleMotor.appliedOutput < 0 && angleAtBottom())) {
            angleMotor.stopMotor()
        }
    }

    fun setExtend(speed: Double) {
        if (extensionInited && extensionSafe()) {
            if (!((speed > 0 && extensionAtTop()) || (speed < 0 && extensionAtBottom()))) {
                extendMotorOne.setVoltage(speed)
            }
        }
    }

    fun setAngle(speed : Double){
        if (angleInited) {
            if (!((speed > 0 && angleAtTop()) || (speed < 0 && angleAtBottom()))) {
                angleMotor.setVoltage(speed)
            }
        }
    }

    fun extensionMeasure(): Double {
        return if (!extensionInited) {
            0.0
        } else {
            extendEncoder.position
        }
    }

    fun angleMeasure(): Double {
        // This basically offsets the angle and allows for it to be negative,
        // so it is guaranteed to be linear on the range it is used
        return (trueAngleEncoder.distance - PhysicalConstants.elevatorFlipOffset).mod(2 * PI) + PhysicalConstants.elevatorFlipOffset - PhysicalConstants.elevatorAngleOffset
    }

    fun extensionAtTop(): Boolean {
        return !upperExtension.get()
    }

    fun extensionAtBottom(): Boolean {
        return !lowerExtension.get()
    }

    fun extensionSafe(): Boolean {
        val angleMeasure = angleMeasure()
        return angleMeasure >= PhysicalConstants.elevatorExtensionMinAngle && angleMeasure <= PhysicalConstants.elevatorExtensionMaxAngle
    }

    fun angleAtTop(): Boolean {
        return angleMeasure() > PhysicalConstants.elevatorAngleTop
    }

    fun angleAtBottom(): Boolean {
        return angleMeasure() < PhysicalConstants.elevatorAngleBottom
    }

    fun stop() {
        extendMotorOne.stopMotor()
        angleMotor.stopMotor()
    }
}
