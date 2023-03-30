package org.sert2521.chargedup2023

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.sert2521.chargedup2023.commands.*
import org.sert2521.chargedup2023.subsystems.Claw
import kotlin.math.*

// This file is too big, it should probably be broken up

class SwerveModuleData(val position: Translation2d, val powerMotorID: Int, val angleMotorID: Int, val angleEncoderID: Int, val angleOffset: Double, val inverted: Boolean)

object PhysicalConstants {
    // Difference of the ys over xs * conversions to meters
    const val elevatorExtensionConversion = (8.375 - 40.5625) / (-1.071426391601562 - 70.07333374023438) / 100.0 * 2.54
    const val elevatorAngleConversion = -2 * PI
    // Difference of the ys over xs
    const val elevatorAngleMotorDistanceConversion = (0.000702229045117 - 1.15102295437296) / (-73.89722442626953 + 0.31786513328552246)
    // Velocity is in rpm so needs / 60
    const val elevatorAngleMotorVelocityConversion = elevatorAngleMotorDistanceConversion / 60.0
    const val elevatorFlipOffset = 0.085541770703388 - PI
    const val elevatorAngleOffset = -0.638255487307134

    const val elevatorExtensionTop = 0.520943999290466
    const val elevatorExtensionBottom = 0.005745772912799999

    const val elevatorAngleTop = 1.19
    const val elevatorExtensionMaxAngle = 1.16
    const val elevatorAngleBottom = 0.005
    const val elevatorAngleMotorBottom = -0.3
    const val elevatorExtensionMinAngle = 0.05

    const val elevatorExtensionDrive = 0.0
    const val elevatorExtensionConeHigh = 0.508
    const val elevatorExtensionCubeHigh = 0.4953
    const val elevatorExtensionMid = 0.1651
    const val elevatorExtensionLow = 0.0
    const val elevatorExtensionConeTippedIntake = 0.0
    const val elevatorExtensionCubeIntake = 0.0
    const val elevatorExtensionConeUpIntake = 0.0
    const val elevatorExtensionSingleSubstation = 0.0

    const val elevatorAngleDrive = 1.19
    const val elevatorAngleConeHigh = 0.66
    const val elevatorAngleCubeHigh = 0.58
    const val elevatorAngleMid = 0.68
    const val elevatorAngleLow = 0.22
    const val elevatorAngleConeTippedIntake = 0.0
    const val elevatorAngleCubeIntake = 0.02
    const val elevatorAngleConeUpIntake = 0.13
    const val elevatorAngleSingleSubstation = 0.685

    // Velocity is in rpm so needs / 60
    const val clawVelocityConversion = 1.0 / 60.0

    const val halfSideLength = 0.286378246381

    // Pi * diameter / gear ratio
    const val powerEncoderMultiplierPosition = PI * 0.1016 / 8.14
    // Velocity is in rpm so needs / 60
    const val powerEncoderMultiplierVelocity = powerEncoderMultiplierPosition / 60.0

    const val angleEncoderMultiplier = 0.01745329251

    val rightPose = Transform3d(Translation3d(0.02872994, -0.3009138, 0.65), Rotation3d(0.0, -0.0873, -0.436))
    val leftPose = Transform3d(Translation3d(0.02872994, 0.3009138, 0.65), Rotation3d(0.0, -0.0873, 0.436))

    val field: AprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField()
    const val fieldLength = 16.54
    const val fieldWidth = 8.02

    init {
        field.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide)
    }

    const val coneAngle = -PI
    // These may be wrong
    val conePointsBlue = listOf(4.97, 3.85, 3.28, 2.17, 1.62, 0.50)
    val conePointsRed: List<Double>
    init {
        val conePointsRedMut = mutableListOf<Double>()
        for (conePointBlue in conePointsBlue) {
            conePointsRedMut.add(fieldWidth - conePointBlue)
        }

        conePointsRed = conePointsRedMut
    }

    const val substationX = fieldLength - 2.48
    val substationFarAngleAtDistance = Pair(PI / 4, 0.3)
    val substationCloseAngleAtDistance = Pair(PI / 2, 0.15)

    const val ledLength = 200

    val ledPurpleHSV = arrayOf(145, 255, 255)

    val ledYellowHSV = arrayOf(10, 255, 255)

    const val cableDriveDistance = 1.0

    // Polar is annoying
    // This takes points and makes them into a bunch of lines in polar coords
    // Then it uses them as boundaries
    // I would add comments if I remembered how this works
    private const val extensionExtra = 0.76
    // There should be no vertical lines or horizontal lines one of them will break the code maybe
    // Something seems to be wrong with y values
    private val safePoints = arrayOf(Pair(0.80,0.0), Pair(0.81,0.4), Pair(1.3,0.7))
    private val safeLineDefinitions = generateLineDefinitions(safePoints)
    private val safeLineBounds = generateLineBounds(safePoints)

    private fun generateLineDefinitions(points: Array<Pair<Double, Double>>): Array<Pair<Double, Double>> {
        val lineDefinitions = mutableListOf<Pair<Double, Double>>()
        for (i in 0 until points.size - 1) {
            val x = points[i].first
            val y = points[i].second
            val m = (y - points[i + 1].second) / (x - points[i + 1].first)
            val mSqrdPlusOne = m.pow(2) + 1
            lineDefinitions.add(Pair((m * x - y) * sqrt(mSqrdPlusOne) / mSqrdPlusOne, atan(1 / m)))
        }

        return lineDefinitions.toTypedArray()
    }

    private fun generateLineBounds(points: Array<Pair<Double, Double>>): Array<Pair<Double, Double>> {
        val lineBounds = mutableListOf<Pair<Double, Double>>()
        for (i in 1 until points.size - 1) {
            val x = points[i].first
            val y = points[i].second
            lineBounds.add(Pair(sqrt(x.pow(2) + y.pow(2)), atan2(y, x)))
        }

        return lineBounds.toTypedArray()
    }

    fun minAngleWithExtension(extension: Double): Double {
        val trueExtension = extension + extensionExtra
        var lineDefinitionIndex = 0
        for (i in safeLineBounds.indices) {
            if (trueExtension > safeLineBounds[i].first) {
                lineDefinitionIndex = i + 1
                break
            }
        }

        val definition = safeLineDefinitions[lineDefinitionIndex]

        if (abs(trueExtension) <= definition.first) {
            return Double.NEGATIVE_INFINITY
        }

        return acos(definition.first / trueExtension) - definition.second
    }

    fun maxExtensionWithAngle(angle: Double): Double {
        var lineDefinitionIndex = 0
        for (i in safeLineBounds.indices) {
            if (angle > safeLineBounds[i].second) {
                lineDefinitionIndex = i + 1
                break
            }
        }

        val definition = safeLineDefinitions[lineDefinitionIndex]

        val max = definition.first / cos(angle + definition.second)
        if (max < 0.0) {
            return Double.POSITIVE_INFINITY
        }

        return max - extensionExtra
    }
}

// Move some of these to config constants (maybe)
object TunedConstants {
    const val elevatorExtensionP = 39.3701
    const val elevatorExtensionI = 0.0
    const val elevatorExtensionD = 0.0

    const val elevatorExtensionG = 1.2

    const val elevatorExtensionMaxV = 2.032
    const val elevatorExtensionMaxA = 3.556

    const val elevatorExtensionTolerance = 0.035

    const val elevatorExtensionMaxAngleTarget = 1.1
    const val elevatorExtensionMinAngleTarget = 0.1

    const val elevatorAngleP = 40.0
    const val elevatorAngleI = 0.0
    const val elevatorAngleD = 0.0

    const val elevatorAngleG = 0.5
    const val elevatorAngleGPerMeter = 0.0

    const val elevatorAngleDownMaxV = 2.0
    const val elevatorAngleDownMaxA = 0.8
    const val elevatorAngleDownMaxAByAngle = 0.8

    const val elevatorAngleUpMaxV = 3.0
    const val elevatorAngleUpMaxA = 4.5

    const val elevatorAngleTolerance = 0.025

    const val elevatorTrustTrueAngleDistance = 0.03
    const val elevatorTrustWrapDistance = 0.07

    const val elevatorPullUpAngleDifference = 0.1

    // Sysid these all
    const val swervePowerS = 0.3
    const val swervePowerV = 3.0
    const val swervePowerA = 0.0

    // Sysid these all
    const val swervePowerP = 2.0
    const val swervePowerI = 0.0
    const val swervePowerD = 0.0

    const val swerveAngleP = 0.5
    const val swerveAngleI = 0.0
    const val swerveAngleD = 0.0

    const val swerveAutoDistanceP = 2.5
    const val swerveAutoDistanceI = 0.0
    const val swerveAutoDistanceD = 0.0

    const val swerveAutoAngleP = 3.0
    const val swerveAutoAngleI = 0.0
    const val swerveAutoAngleD = 0.0

    const val swerveAlignDistanceP = 1.8
    const val swerveAlignDistanceI = 0.0
    const val swerveAlignDistanceD = 0.0

    const val swerveConeAlignAngleP = 2.5
    const val swerveConeAlignAngleI = 0.0
    const val swerveConeAlignAngleD = 0.0

    const val swerveSubstationAlignAngleP = 3.0
    const val swerveSubstationAlignAngleI = 0.0
    const val swerveSubstationAlignAngleD = 0.0

    const val standardFilterTaps = 20
    const val overCableFilterTaps = 4

    const val balanceSpeed = 0.2
    const val balanceAngleSignificantRate = 0.15
    const val balanceAngleTolerance = 0.04

    const val balanceAngleStart = 0.1
    const val balanceDriveUpSpeed = 1.2

    const val cableTipLimit = 0.1

    const val visionConePositionTolerance = 0.01
    const val visionConeAngleTolerance = 0.01

    const val visionSubstationPositionTolerance = 0.02
    const val visionSubstationAngleTolerance = 0.02

    val encoderDeviations: Matrix<N3, N1> = MatBuilder(Nat.N3(), Nat.N1()).fill(1.0, 1.0, 0.01)
    val defaultVisionDeviations: Matrix<N3, N1> = MatBuilder(Nat.N3(), Nat.N1()).fill(1.0, 1.0, 100.0)
    val alignVisionDeviations: Matrix<N3, N1> = MatBuilder(Nat.N3(), Nat.N1()).fill(3.0, 3.0, 0.5)

    const val ledsRainbowA = 2.0
    const val ledsRainbowB = 1.1
    const val ledsRainbowC = 0.7
    const val ledsRainbowD = 3.8
    const val ledsRainbowE = 1.5

    const val clawIntakeTryPower = 0.35
    const val clawOuttakeTryPower = 0.35
    const val clawIntakeStoppedSpeed = 0.25

    const val clawDebounce = 0.25
}

object ConfigConstants {
    const val extensionResetVoltage = -1.0
    const val angleInitAngle = 1.175
    const val angleResetVoltage = 5.5
    const val resetAngle = 0.98

    const val drivetrainOptimized = true

    // Add actual joystick deadband
    const val powerDeadband = 0.075
    const val rotDeadband = 0.075

    const val driveSpeed = 3.5
    const val driveSpeedup = 2.75
    const val rotSpeed = 3.5
    const val rotSpeedup = 2.75

    const val driveSpeedupChangeSpeed = 0.8
    const val driveSlowdownChangeSpeed = 0.4

    const val preBrownOutVoltage = 8.25
    const val preLEDBrownOutVoltage = 11.0

    // Drive needs to be lower for consistency
    val eventMap = mapOf("Elevator Drive" to SetElevator(PhysicalConstants.elevatorExtensionDrive, PhysicalConstants.elevatorAngleDrive, true),
        "Elevator Cone High" to SetElevator(PhysicalConstants.elevatorExtensionConeHigh, PhysicalConstants.elevatorAngleConeHigh, true).andThen(SetElevator(PhysicalConstants.elevatorExtensionConeHigh, PhysicalConstants.elevatorAngleConeHigh, false).withTimeout(0.25)),
        "Claw Outtake" to ClawIntake(1.0).withTimeout(0.4),
        "Elevator Cube Intake" to SetElevator(PhysicalConstants.elevatorExtensionCubeIntake, PhysicalConstants.elevatorAngleCubeIntake, true),
        "Elevator Cone Intake" to SetElevator(PhysicalConstants.elevatorExtensionConeTippedIntake, PhysicalConstants.elevatorAngleConeTippedIntake, true),
        "Claw Cube Intake" to ClawIntake(-0.7),
        "Claw Stop" to InstantCommand({  }, Claw),
        "Elevator Cube High" to SetElevator(PhysicalConstants.elevatorExtensionCubeHigh, PhysicalConstants.elevatorAngleCubeHigh, true),
        "Elevator Half" to SetElevator(PhysicalConstants.elevatorExtensionSingleSubstation, PhysicalConstants.elevatorAngleSingleSubstation, true),
        "Claw Cube Outtake" to ClawIntake(0.75).withTimeout(0.4),
        "Drive Back Onto Charge Station" to SequentialCommandGroup(OntoChargeStation(Translation2d(-1.0, 0.0)), DriveUpChargeStation().withTimeout(1.6), Balance()),
        "Drive Front Onto Charge Station" to SequentialCommandGroup(OntoChargeStation(Translation2d(1.0, 0.0)), DriveUpChargeStation().withTimeout(1.6), Balance()))
        //"Align Vision Cube" to VisionAlignAuto(4.46, 2.0).withTimeout(2.0))

    private val autoConstraints = PathConstraints(1.8, 1.7)
    private val fastAutoConstraints = PathConstraints(2.6, 2.3)
    // Slow this down more (maybe)
    private val fastishAutoConstraints = PathConstraints(2.2, 2.0)

    private val pathsData = arrayOf(
        Pair("Cable Balance", autoConstraints), // Yes
        Pair("Cable 1.5 Balance", autoConstraints), // Yes
        Pair("Cable 1.5", autoConstraints), // Yes
        Pair("Cable 1", autoConstraints), // Yes
        Pair("Cable 2", autoConstraints), // Mostly
        Pair("Forward", autoConstraints), // Yes
        Pair("No Cable Balance", autoConstraints), // Yes
        Pair("No Cable 1.5 Balance", autoConstraints), // Yes
        Pair("No Cable 1", autoConstraints), // Yes
        Pair("No Cable 2.5 Cube", fastishAutoConstraints), // Mostly
        Pair("No Cable 2.5 Cone", fastishAutoConstraints), // Mostly
        Pair("No Cable 2", autoConstraints), // Yes
        //Pair("No Cable 2 Balance Far", fastAutoConstraints), // No
        Pair("No Cable 2 Balance Near", fastAutoConstraints)) // Mostly

    val paths: Array<Pair<String, List<PathPlannerTrajectory>>>
    init {
        val pathsList = mutableListOf<Pair<String, List<PathPlannerTrajectory>>>()

        for (data in pathsData) {
            pathsList.add(Pair(data.first, PathPlanner.loadPathGroup(data.first, data.second)))
        }

        paths = pathsList.toTypedArray()
    }
}

object ElectronicIDs {
    const val clawMotorId = 9
    const val elevatorMotorOne = 6
    const val elevatorMotorTwo = 10
    const val elevatorAngleMotor = 3

    const val elevatorEncoder = 5

    const val elevatorUpperExtension = 2
    const val elevatorLowerExtension = 1

    val swerveModuleData = listOf(
        SwerveModuleData(Translation2d(PhysicalConstants.halfSideLength, -PhysicalConstants.halfSideLength), 4, 5, 14, -2.27 + PI / 2 + 4.62 + 1.54 - PI / 2, true),
        SwerveModuleData(Translation2d(-PhysicalConstants.halfSideLength, -PhysicalConstants.halfSideLength), 1, 2, 16, -1.63 - PI + 4.79 + 1.61 - PI / 2, true),
        SwerveModuleData(Translation2d(PhysicalConstants.halfSideLength, PhysicalConstants.halfSideLength), 12, 11, 13, -0.76 + PI / 2 - 1.43 + 1.57 - PI / 2, true),
        SwerveModuleData(Translation2d(-PhysicalConstants.halfSideLength, PhysicalConstants.halfSideLength), 7, 8, 15, -4.10 - PI / 2 + 5.12 + 1.75 - PI / 2, true))

    const val ledId = 0

    val camData = listOf(Pair("Right2", PhysicalConstants.rightPose), Pair("Left2", PhysicalConstants.leftPose))
}
