package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import org.sert2521.chargedup2023.Input
import org.sert2521.chargedup2023.Output
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain
import java.lang.Math.PI

class VisionAlignSubstation : JoystickCommand() {
    private val positionPID = ProfiledPIDController(TunedConstants.swerveAlignDistanceP, TunedConstants.swerveAlignDistanceI, TunedConstants.swerveAlignDistanceD, TrapezoidProfile.Constraints(TunedConstants.swerveAlignV, TunedConstants.swerveAlignA))
    private val anglePID = PIDController(TunedConstants.swerveAlignAngleP, TunedConstants.swerveAlignAngleI, TunedConstants.swerveAlignAngleD)

    init {
        addRequirements(Drivetrain)

        anglePID.enableContinuousInput(0.0, 2 * PI)

        positionPID.setTolerance(TunedConstants.visionPositionTolerance)
        anglePID.setTolerance(TunedConstants.visionAngleTolerance)
    }

    override fun initialize() {
        super.initialize()

        Drivetrain.setVisionStandardDeviations()
        positionPID.reset(Drivetrain.getPose().y, Drivetrain.deltaPose.y)
        anglePID.reset()
    }

    override fun execute() {
        val pose = Drivetrain.getVisionPose()

        val color = Input.getColor()
        val xTarget = PhysicalConstants.colorToSubstation[color]
        val farAngleAtDistance = PhysicalConstants.colorToSubstationFarAngleAtDistance[color]
        val closeAngleAtDistance = PhysicalConstants.colorToSubstationCloseAngleAtDistance[color]
        if (xTarget != null && farAngleAtDistance != null && closeAngleAtDistance != null) {
            val t = clamp((pose.x - closeAngleAtDistance.second) / (farAngleAtDistance.second - closeAngleAtDistance.second), 0.0, 1.0)
            val angleTarget = t * (farAngleAtDistance.first - closeAngleAtDistance.first) + closeAngleAtDistance.first

            // Moving the x on the stick will affect the rate of change of the y
            Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(positionPID.calculate(pose.x, xTarget), readJoystick().y, anglePID.calculate(pose.rotation.radians, angleTarget), pose.rotation))
        } else {
            positionPID.reset(pose.y, Drivetrain.deltaPose.y)
            anglePID.reset()

            Drivetrain.stop()
        }

        Output.visionHappy = positionPID.atSetpoint() && anglePID.atSetpoint()
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.setVisionAlignDeviations()
        Output.visionHappy = false
        Drivetrain.stop()
    }
}