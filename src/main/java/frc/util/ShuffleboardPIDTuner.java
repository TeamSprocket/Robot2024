// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class ShuffleboardPIDTuner extends SubsystemBase{
    private static ShuffleboardTab tab = Shuffleboard.getTab("PIDTuner2");
    private static ArrayList<GenericEntry> values = new ArrayList<GenericEntry>();
    private static ArrayList<String> tags = new ArrayList<String>();
    private static ArrayList<Double> defaultVals = new ArrayList<Double>();

    public static void addSlider(String widgetName, double min, double max, double defaultValue) {
        values.add(
        tab.add(widgetName, defaultValue)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", min, "max", max))
            .getEntry());
        tags.add(widgetName);
        defaultVals.add(defaultValue);
    }

    public static double get(String widgetName) {
        int entryInd = tags.indexOf(widgetName);
        double defaultVal = defaultVals.get(entryInd);
        return values.get(entryInd).getDouble(defaultVal);
    }

}
