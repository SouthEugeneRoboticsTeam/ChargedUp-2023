package org.sert2521.chargedup2023.subsystems

import com.revrobotics.AbsoluteEncoder
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycle
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.chargedup2023.ElectronicIDs
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants
import kotlin.math.PI

object Elevator : SubsystemBase() {
    private val extendMotorOne = CANSparkMax(ElectronicIDs.elevatorMotorOne, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val extendMotorTwo = CANSparkMax(ElectronicIDs.elevatorMotorTwo, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val extendEncoder = extendMotorOne.encoder

    private val upperExtension = DigitalInput(ElectronicIDs.elevatorUpperExtension)
    private val lowerExtension = DigitalInput(ElectronicIDs.elevatorLowerExtension)

    var extensionInited = false
        private set

    private val angleMotor = CANSparkMax(ElectronicIDs.elevatorAngleMotor, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val trueAngleEncoder = DutyCycleEncoder(ElectronicIDs.elevatorEncoder)

    init {
        extendMotorOne.idleMode = CANSparkMax.IdleMode.kBrake
        extendMotorTwo.idleMode = CANSparkMax.IdleMode.kBrake
        angleMotor.idleMode = CANSparkMax.IdleMode.kBrake

        extendMotorTwo.follow(extendMotorOne, true)

        extendEncoder.positionConversionFactor = PhysicalConstants.elevatorExtensionConversion

        trueAngleEncoder.distancePerRotation = PhysicalConstants.elevatorAngleConversion
    }

    override fun periodic() {
        val atTopExtension = extensionAtTop()
        val atBottomExtension = extensionAtBottom()
        val safe = extensionSafe()

        if (atTopExtension) {
            extendEncoder.position = PhysicalConstants.elevatorExtensionTop
            extensionInited = true
        }

        if (atBottomExtension) {
            extendEncoder.position = PhysicalConstants.elevatorExtensionBottom
            extensionInited = true
        }

        if (!extensionInited && safe) {
            extendMotorOne.set(TunedConstants.extensionResetSpeed)
        }

        if (!safe || (extendMotorOne.get() > 0 && atTopExtension) || (extendMotorOne.get() < 0 && atBottomExtension)) {
            extendMotorOne.stopMotor()
        }

        if ((angleMotor.get() > 0 && angleAtTop()) || (angleMotor.get() < 0 && angleAtBottom())) {
            angleMotor.stopMotor()
        }
    }

    fun setExtend(speed: Double) {
        if (extensionInited && extensionSafe()) {
            if (!((speed > 0 && extensionAtTop()) || (speed < 0 && extensionAtBottom()))) {
                extendMotorOne.set(speed)
            }
        }
    }

    fun setAngle(speed : Double){
        if (!((speed > 0 && angleAtTop()) || (speed < 0 && angleAtBottom()))) {
            angleMotor.set(speed)
        }
    }

    fun extensionMeasure(): Double {
        return extendEncoder.position
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
        return angleMeasure() >= PhysicalConstants.elevatorExtensionMinAngle
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
