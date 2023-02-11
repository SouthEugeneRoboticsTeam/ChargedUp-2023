package org.sert2521.chargedup2023.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.chargedup2023.ElectronicIDs
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants

//The Dominion of Milo.

object Elevator : SubsystemBase() {
    private val extendMotorOne = CANSparkMax(ElectronicIDs.elevatorMotorOne, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val extendMotorTwo = CANSparkMax(ElectronicIDs.elevatorMotorTwo, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val extendEncoder = extendMotorOne.encoder

    private val upperExtension = DigitalInput(ElectronicIDs.elevatorUpperExtension)
    private val lowerExtension = DigitalInput(ElectronicIDs.elevatorLowerExtension)

    private var extensionInited = false

    private val angleMotor = CANSparkMax(ElectronicIDs.elevatorAngleMotor, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val trueAngleEncoder = Encoder(ElectronicIDs.elevatorEncoderA, ElectronicIDs.elevatorEncoderB)
    private var trueAngleOffset = 0.0

    //private val upperAngle = DigitalInput(ElectronicIDs.elevatorUpperAngle)
    private val lowerAngle = DigitalInput(ElectronicIDs.elevatorLowerAngle)

    private var angleInited = false

    init {
        extendMotorOne.idleMode = CANSparkMax.IdleMode.kBrake
        extendMotorTwo.idleMode = CANSparkMax.IdleMode.kBrake
        angleMotor.idleMode = CANSparkMax.IdleMode.kBrake

        extendMotorTwo.follow(extendMotorOne, true)

        extendEncoder.positionConversionFactor = PhysicalConstants.elevatorExtensionConversion

        trueAngleEncoder.distancePerPulse = PhysicalConstants.elevatorAngleConversion
    }

    override fun periodic() {
        val atTopExtension = extensionAtTop()
        val atBottomExtension = extensionAtBottom()

        if (atTopExtension) {
            extendEncoder.position = PhysicalConstants.elevatorExtensionTop
            extensionInited = true
        }

        if (atBottomExtension) {
            extendEncoder.position = PhysicalConstants.elevatorExtensionBottom
            extensionInited = true
        }

        if (!extensionInited) {
            extendMotorOne.set(TunedConstants.extensionResetSpeed)
        }

        if ((extendMotorOne.get() > 0 && atTopExtension)) {
            extendMotorOne.stopMotor()
        }

        if ((extendMotorOne.get() < 0 && atBottomExtension)) {
            extendMotorOne.stopMotor()
        }

        val atBottomAngle = angleAtBottom()

        if (atBottomAngle) {
            trueAngleOffset = PhysicalConstants.elevatorAngleBottom - trueAngleEncoder.distance
            angleInited = true
        }

        if (!angleInited) {
            angleMotor.set(TunedConstants.angleResetSpeed)
        }

        if ((angleMotor.get() > 0 && angleAtTop())) {
            angleMotor.stopMotor()
        }

        if ((angleMotor.get() < 0 && atBottomAngle)) {
            angleMotor.stopMotor()
        }
    }

    fun setExtend(speed: Double) {
        if (extensionInited) {
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
        return trueAngleEncoder.distance + trueAngleOffset
    }

    fun extensionAtTop(): Boolean {
        return !upperExtension.get()
    }

    fun extensionAtBottom(): Boolean {
        return !lowerExtension.get()
    }

    fun angleAtTop(): Boolean {
        return angleMeasure() > PhysicalConstants.elevatorAngleTop
    }

    fun angleAtBottom(): Boolean {
        return !lowerAngle.get()
    }

    fun stop() {
        extendMotorOne.stopMotor()
        angleMotor.stopMotor()
    }
}
