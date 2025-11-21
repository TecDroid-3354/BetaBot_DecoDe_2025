package org.firstinspires.ftc.teamcode.subsystems.indexer

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.command.WaitCommand
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.indexer.Slot.Slot
import org.firstinspires.ftc.teamcode.subsystems.indexer.Slot.SlotConfig
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerConstants.Ids
import org.firstinspires.ftc.teamcode.utils.colorSensor.ColorSensorEx.DetectedColor
import java.util.Optional

enum class MotifPatterns(val pattern: List<DetectedColor>) {
    PURPLE_PURPLE_GREEN(listOf(DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN)),
    PURPLE_GREEN_PURPLE(listOf(DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE)),
    GREEN_PURPLE_PURPLE(listOf(DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE)),
    GREEN_GREEN_PURPLE(listOf(DetectedColor.GREEN, DetectedColor.GREEN, DetectedColor.PURPLE))
}

class Indexer(val hw: HardwareMap, val telemetry: Telemetry) : SubsystemBase() {
    private lateinit var frontSlot: Slot
    private lateinit var leftSlot: Slot
    private lateinit var rightSlot: Slot
    private lateinit var slotList: Array<Slot>
    private var slotOrder: Array<String> = arrayOf()

    private var lastSlot: String? = null

    init {
        frontSlot = Slot(
            SlotConfig(Ids.frontServo, Ids.colorSensorFront, "front"),
            hw,
            telemetry)

        leftSlot = Slot(
            SlotConfig(Ids.leftServo, Ids.colorSensorLeft, "left"),
            hw,
            telemetry)

        rightSlot = Slot(
            SlotConfig(Ids.rightServo, Ids.colorSensorRight, "right"),
            hw,
            telemetry)

        slotList = arrayOf(frontSlot, leftSlot, rightSlot)
    }

    fun feedShooter(): SequentialCommandGroup {
        return SequentialCommandGroup(
            feedCMD(),
            feedCMD(),
            feedCMD(),
        )
    }

    fun feedShooter(motifPatterns: MotifPatterns): SequentialCommandGroup {
        for (color in motifPatterns.pattern) {
            for ((index, slot) in slotList.withIndex()) {
                if (slot.getDetectedColor() == color) {
                    slotOrder.fill(slot.config.slotId, index)

                }
            }
        }

        return SequentialCommandGroup(
            feedCMD(motifPatterns.pattern[0]),
            WaitCommand(500),
            feedCMD(motifPatterns.pattern[1]),
            WaitCommand(500),
            feedCMD(motifPatterns.pattern[2]),
        )
    }

    fun feedAllShooter(): SequentialCommandGroup {
        return SequentialCommandGroup(
            feedCMD(slotList[0]),
            feedCMD(slotList[1]),
            feedCMD(slotList[2]),
        )
    }

    private fun feedCMD(color: DetectedColor): Command {
        return SequentialCommandGroup(
            InstantCommand({ slot.feed() }),
            WaitCommand(1000),
            InstantCommand({ slot.home() }),
            WaitCommand(1000)
        )
    }

    private fun feedCMD(): Command {
        for (slot in slotList) {
            if (slot.getDetectedColor() != DetectedColor.UNKNOWN) {
                return SequentialCommandGroup(
                    InstantCommand({ slot.feed() }),
                    WaitCommand(1000),
                    InstantCommand({ slot.home() }),
                    WaitCommand(1000)
                )
            }
        }

        return InstantCommand()
    }

    private fun feedCMD(slot: Slot): Command {
            return SequentialCommandGroup(
                InstantCommand({ slot.feed() }),
                WaitCommand(1000),
                InstantCommand({ slot.home() }),
                WaitCommand(1000))
    }

    override fun periodic() {
        telemetry.addData("Slot1", slotList[0].getDetectedColor())
        telemetry.addData("Slot2", slotList[1].getDetectedColor())
        telemetry.addData("Slot3", slotList[2].getDetectedColor())
    }

}