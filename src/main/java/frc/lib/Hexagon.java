// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

/** Add your docs here. */
    import java.awt.Point;

public class Hexagon {

    private Point center;
    private double sideLength;

    public Hexagon(Point center, double sideLength) {
        this.center = center;
        this.sideLength = sideLength;
    }

    public boolean contains(Point p) {
        // Check if point is within the bounding box of the hexagon
        if (Math.abs(p.x - center.x) > sideLength * Math.sqrt(3) / 2 || 
            Math.abs(p.y - center.y) > sideLength) {
            return false;
        }

        // Check if point is within each of the six triangular sections
        for (int i = 0; i < 6; i++) {
            double angle = Math.PI / 3 * i;
            double x = center.x + sideLength * Math.cos(angle);
            double y = center.y + sideLength * Math.sin(angle);

            // Calculate cross product to determine if point is on the correct side of the edge
            if ((p.x - center.x) * (y - center.y) - (p.y - center.y) * (x - center.x) > 0) {
                return false;
            }
        }
        return true;
    }
}

