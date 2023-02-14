package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Elevator

class SetElevator(private val extension: Double, private val angle: Double, private val ends: Boolean) : CommandBase() {
    private val anglePID = ProfiledPIDController(TunedConstants.elevatorAngleP, TunedConstants.elevatorAngleI, TunedConstants.elevatorAngleD, TrapezoidProfile.Constraints(TunedConstants.elevatorAngleMaxV, TunedConstants.elevatorAngleMaxA))

    private val extensionPID = ProfiledPIDController(TunedConstants.elevatorExtensionP, TunedConstants.elevatorExtensionI, TunedConstants.elevatorExtensionD, TrapezoidProfile.Constraints(TunedConstants.elevatorExtensionMaxV, TunedConstants.elevatorExtensionMaxA))

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
        val angleTarget = if (angle >= TunedConstants.elevatorExtensionMinAngleTarget || extensionPID.atSetpoint()) {
            angle
        } else {
            TunedConstants.elevatorExtensionMinAngleTarget
        }

        Elevator.setAngle(anglePID.calculate(Elevator.angleMeasure(), angleTarget) + TunedConstants.elevatorAngleG + Elevator.extensionMeasure() * TunedConstants.elevatorAngleGPerMeter)

        val extensionTarget = if (Elevator.extensionSafe()) {
            extension
        } else {
            Elevator.extensionMeasure()
        }

        Elevator.setExtend(extensionPID.calculate(Elevator.extensionMeasure(), extensionTarget))
    }

    override fun isFinished(): Boolean {
        return ends && anglePID.atGoal() && extensionPID.atGoal()
    }

    override fun end(interrupted: Boolean) {
        Elevator.stop()
    }
}