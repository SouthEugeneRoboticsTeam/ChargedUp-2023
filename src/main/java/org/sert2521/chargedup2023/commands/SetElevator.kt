package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Elevator
import kotlin.math.max
import kotlin.math.min
import kotlin.math.cos

// Maybe have very small feedforward especially angle which is about 0.02rad below goal
class SetElevator(private val extension: Double, private val angle: Double, private val ends: Boolean) : CommandBase() {
    private val anglePID = ProfiledPIDController(TunedConstants.elevatorAngleP, TunedConstants.elevatorAngleI, TunedConstants.elevatorAngleD, TrapezoidProfile.Constraints(TunedConstants.elevatorAngleMaxV, TunedConstants.elevatorAngleMaxA))

    private val extensionPID = ProfiledPIDController(TunedConstants.elevatorExtensionP, TunedConstants.elevatorExtensionI, TunedConstants.elevatorExtensionD, TrapezoidProfile.Constraints(TunedConstants.elevatorExtensionMaxV, TunedConstants.elevatorExtensionMaxA))

    private val angleTooLow = angle > TunedConstants.elevatorExtensionMinAngleTarget
    private val angleTooHigh = angle < TunedConstants.elevatorExtensionMaxAngleTarget
    private val angleSafe = (!angleTooLow && !angleTooHigh)

    init {
        addRequirements(Elevator)

        anglePID.setTolerance(TunedConstants.elevatorAngleTolerance)
        extensionPID.setTolerance(TunedConstants.elevatorExtensionTolerance)
    }

    override fun initialize() {
        anglePID.reset(Elevator.angleMeasure())

        extensionPID.reset(Elevator.extensionMeasure())
    }

    override fun execute() {
        val angleTarget = if (angleSafe || extensionPID.atSetpoint()) {
            angle
        } else if (angleTooLow) {
            TunedConstants.elevatorExtensionMinAngleTarget
        } else {
            TunedConstants.elevatorExtensionMaxAngleTarget
        }

        val extensionMeasure = Elevator.extensionMeasure()
        val extensionTarget = if (Elevator.extensionSafe()) {
            extension
        } else {
            Elevator.extensionMeasure()
        }

        val angleMeasure = Elevator.angleMeasure()
        val anglePIDResult = anglePID.calculate(Elevator.angleMeasure(), angleTarget)
        val g = cos(anglePID.setpoint.position) * (TunedConstants.elevatorAngleG + extensionMeasure * TunedConstants.elevatorAngleGPerMeter)
        Elevator.setAngle(anglePIDResult + g)
        Elevator.setExtend(extensionPID.calculate(extensionMeasure, min(extensionTarget, PhysicalConstants.maxExtensionWithAngle(extension))))
    }

    override fun isFinished(): Boolean {
        return ends && anglePID.atGoal() && extensionPID.atGoal()
    }

    override fun end(interrupted: Boolean) {
        Elevator.stop()
    }
}