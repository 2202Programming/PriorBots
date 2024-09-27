/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

/**
 * Add your docs here.
 */
public class Stick {

    public static void log(String message) {
        // StackTraceElement lastMethod = Thread.currentThread().getStackTrace()[2];
        // String packageName = getPackageName(lastMethod);
        System.out.print("[Stick]: " + message + "\n");
    }

    /**
     * Gets a friendly package name for a StackTraceElement
     * 
     * @param element Element to get name for
     * @return Friendly Name
     */
    protected static String getPackageName(StackTraceElement element) {

        // Build the package and method string
        String fullPackageName = element.getClassName();
        String[] packageSegments = fullPackageName.split("[.]");
        StringBuilder packageNameBuilder = new StringBuilder();

        // Handle the package name being too short
        if (packageSegments.length <= 3) {

            // Just build the full package name
            packageNameBuilder.append(fullPackageName);

        } else {
            // The idea here is to turn a package name that might look like:
            // io.github.frc5024.y2020.darthraider.commands.autonomous.actions.cells.SetShooterOutput
            // Into one that looks like:
            // io...cells.SetShooterOutput

            // Append the root package
            packageNameBuilder.append(packageSegments[0]);

            // Append the seperator
            packageNameBuilder.append("...");

            // Append the class's parent package
            packageNameBuilder.append(packageSegments[packageSegments.length - 2]);
            packageNameBuilder.append(".");

            // Append the class name
            packageNameBuilder.append(packageSegments[packageSegments.length - 1]);
        }

        // Get the method name
        String methodName = element.getMethodName();

        return String.format("%s::%s()", packageNameBuilder.toString(), methodName.toString());

    }
}
