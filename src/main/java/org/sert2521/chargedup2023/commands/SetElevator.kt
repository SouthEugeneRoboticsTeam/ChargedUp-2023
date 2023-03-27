package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Elevator
import kotlin.math.*

class SetElevator(private val extension: Double, private val angle: Double, private val ends: Boolean) : CommandBase() {
    private val extensionPID = ProfiledPIDController(TunedConstants.elevatorExtensionP, TunedConstants.elevatorExtensionI, TunedConstants.elevatorExtensionD, TrapezoidProfile.Constraints(TunedConstants.elevatorExtensionMaxV, TunedConstants.elevatorExtensionMaxA))
    // Could use continuous input instead of having negative rotations in the elevator code
    private val anglePID = ProfiledPIDController(TunedConstants.elevatorAngleP, TunedConstants.elevatorAngleI, TunedConstants.elevatorAngleD, TrapezoidProfile.Constraints(TunedConstants.elevatorAngleUpMaxV, TunedConstants.elevatorAngleUpMaxA))

    private val angleTooLow = angle < TunedConstants.elevatorExtensionMinAngleTarget
    private val angleTooHigh = angle > TunedConstants.elevatorExtensionMaxAngleTarget
    private val angleSafe = !angleTooLow && !angleTooHigh

    private var atSetpoint = false

    init {
        addRequirements(Elevator)
    }

    override fun initialize() {
        anglePID.reset(Elevator.angleMeasure())
        extensionPID.reset(Elevator.extensionMeasure())
    }

    // Clamping the target is kind of just in case
    override fun execute() {
        val angleTarget = if (angleSafe || extensionPID.atGoal()) {
            angle
        } else if (angleTooLow) {
            TunedConstants.elevatorExtensionMinAngleTarget
        } else {
            TunedConstants.elevatorExtensionMaxAngleTarget
        }

        val extensionMeasure = Elevator.extensionMeasure()
        val angleMeasure = Elevator.angleMeasure()

        var safeAngleTarget = clamp(max(angleTarget, PhysicalConstants.minAngleWithExtension(extensionMeasure)), PhysicalConstants.elevatorAngleBottom, PhysicalConstants.elevatorAngleTop)
        if (safeAngleTarget < angleMeasure) {
            if (Elevator.angleSusness() >= TunedConstants.elevatorSusLimit) {
                safeAngleTarget = angleMeasure
            }

            anglePID.setConstraints(TrapezoidProfile.Constraints(TunedConstants.elevatorAngleDownMaxV, TunedConstants.elevatorAngleDownMaxA + TunedConstants.elevatorAngleDownMaxAByAngle * cos(angleMeasure)))
        } else {
            anglePID.setConstraints(TrapezoidProfile.Constraints(TunedConstants.elevatorAngleUpMaxV, TunedConstants.elevatorAngleUpMaxA))
        }

        val anglePIDResult = anglePID.calculate(angleMeasure, safeAngleTarget)
        val angleG = cos(safeAngleTarget) * (TunedConstants.elevatorAngleG + extensionMeasure * TunedConstants.elevatorAngleGPerMeter)
        Elevator.setAngle(anglePIDResult + angleG)

        val extensionTarget = if (Elevator.extensionSafe()) {
            extension
        } else {
            extensionMeasure
        }

        val safeExtensionTarget = clamp(min(extensionTarget, PhysicalConstants.maxExtensionWithAngle(angleMeasure)), PhysicalConstants.elevatorExtensionBottom, PhysicalConstants.elevatorExtensionTop)
        val extensionPIDResult = extensionPID.calculate(extensionMeasure, safeExtensionTarget)
        val extensionG = sin(safeAngleTarget) * (TunedConstants.elevatorExtensionG)
        Elevator.setExtend(extensionPIDResult + extensionG)

        if (ends) {
            atSetpoint = abs(angle - angleMeasure) <= TunedConstants.elevatorAngleTolerance || abs(extension - extensionMeasure) <= TunedConstants.elevatorExtensionTolerance
        }
    }

    override fun isFinished(): Boolean {
        return atSetpoint
    }

    override fun end(interrupted: Boolean) {
        Elevator.stop()
    }
}