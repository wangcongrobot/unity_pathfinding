using UnityEngine;
using System.Collections.Generic;
using Pathfinding;

public class ModifierTest : MonoModifier {
    public override int Order { get { return 60; } }

    public int iterations = 5;

    public int subdivisions = 2;

    public override void Apply (Path path) {
        if (path.error || path.vectorPath == null || path.vectorPath.Count <= 2) {
            return;
        }
        // Subdivisions should not be less than zero
        subdivisions = Mathf.Max(subdivisions, 0);

        // Prevent unknowing users from entering bad values
        if (subdivisions > 12) {
            Debug.LogWarning("Subdividing a path more than 12 times is quite a lot, it might cause memory problems and it will certainly slow the game down.\n" +
                "When this message is logged, no smoothing will be applied");
            subdivisions = 12;
            return;
        }

        // Create a new list to hold the smoothed path
        List<Vector3> newPath = new List<Vector3>();
        List<Vector3> originalPath = path.vectorPath;

        // One segment (line) in the original array will be subdivided to this number of smaller segments
        int subSegments = (int)Mathf.Pow(2, subdivisions);
        float fractionPerSegment = 1F / subSegments;
        for (int i = 0; i < originalPath.Count - 1; i++) {
            for (int j = 0; j < subSegments; j++) {
                // Use Vector3.Lerp to place the points at their correct positions along the line
                newPath.Add(Vector3.Lerp(originalPath[i], originalPath[i+1], j*fractionPerSegment));
            }
        }

        // Add the last point
        newPath.Add(originalPath[originalPath.Count-1]);

        // Smooth the path [iterations] number of times
        for (int it = 0; it < iterations; it++) {
            // Loop through all points except the first and the last
            for (int i = 1; i < newPath.Count-1; i++) {
                // Set the new point to the average of the current point and the two adjacent points
                Vector3 newpoint = (newPath[i] + newPath[i-1] + newPath[i+1]) / 3F;
                newPath[i] = newpoint;
            }
        }

        // Assign the new path to the p.vectorPath field
        path.vectorPath = newPath;
    }
}