# Examples

- AstarAI.cs
```C#
using UnityEngine;
// Note this line, if it is left out, the script won't know that the class 'Path' exists and it will throw compiler errors
// This line should always be present at the top of scripts which use pathfinding
using Pathfinding;

public class AstarAI : MonoBehaviour {
    // 目标位置
    public Transform targetPosition;

    private Seeker seeker;
    private CharacterController controller;

    // 计算出来的路线
    public Path path;

    // 移动速度
    public float speed = 2;

    public float nextWaypointDistance = 3;

    // 当前点
    private int currentWaypoint = 0;

    public float repathRate = 0.5f;
    private float lastRepath = float.NegativeInfinity;

    public bool reachedEndOfPath;

    // 初始化
    public void Start () {
        seeker = GetComponent<Seeker>();
        // If you are writing a 2D game you can remove this line
        // and use the alternative way to move sugggested further below.
        controller = GetComponent<CharacterController>();
    }

    // 寻路结束
    public void OnPathComplete (Path p) {
        Debug.Log("A path was calculated. Did it fail with an error? " + p.error);

        // Path pooling. To avoid unnecessary allocations paths are reference counted.
        // Calling Claim will increase the reference count by 1 and Release will reduce
        // it by one, when it reaches zero the path will be pooled and then it may be used
        // by other scripts. The ABPath.Construct and Seeker.StartPath methods will
        // take a path from the pool if possible. See also the documentation page about path pooling.
        p.Claim(this);
        if (!p.error) {
            if (path != null) path.Release(this);
            path = p;
            // Reset the waypoint counter so that we start to move towards the first point in the path
            currentWaypoint = 0;
        } else {
            p.Release(this);
        }
    }

    // 寻路更新
    // 每帧我们都会做几件事：
    // 1. 检查是否有要遵循的计算路径。由于路径请求是异步的，因此可能需要花费几帧（通常是一帧）来计算路径。
    // 2. 检查agent是否靠近其当前要行驶的航路点，如果是，则切换到下一个航路点并重复检查。
    // 3. 为了计算移动方式，我们获取当前航路点的坐标，然后从中减去当前位置。这将为我们提供指向航路点的矢量。我们对该向量进行归一化以使其长度为1，否则我们移动得越快将越远离原先的航路点。
    // 4. 将该矢量乘以预设速度以获得实际速度。
    // 5. 通过使用CharacterController.SimpleMove方法来移动agent。
    public void Update () {

        // 没有找到路
        if (path == null) {
            // We have no path to follow yet, so don't do anything
            return;
        }

        if (Time.time > lastRepath + repathRate && seeker.IsDone()) {
            lastRepath = Time.time;

            // Start a new path to the targetPosition, call the the OnPathComplete function
            // when the path has been calculated (which may take a few frames depending on the complexity)
            // 计算路径: 开始位置,结束位置,回调函数
            seeker.StartPath(transform.position, targetPosition.position, OnPathComplete);
        }


        // Check in a loop if we are close enough to the current waypoint to switch to the next one.
        // We do this in a loop because many waypoints might be close to each other and we may reach
        // several of them in the same frame.
        reachedEndOfPath = false;
        // The distance to the next waypoint in the path
        float distanceToWaypoint;
        while (true) {
            // If you want maximum performance you can check the squared distance instead to get rid of a
            // square root calculation. But that is outside the scope of this tutorial.
            distanceToWaypoint = Vector3.Distance(transform.position, path.vectorPath[currentWaypoint]);
            if (distanceToWaypoint < nextWaypointDistance) {
                // Check if there is another waypoint or if we have reached the end of the path
                if (currentWaypoint + 1 < path.vectorPath.Count) {
                    currentWaypoint++;
                } else {
                    // Set a status variable to indicate that the agent has reached the end of the path.
                    // You can use this to trigger some special code if your game requires that.
                    reachedEndOfPath = true;
                    break;
                }
            } else {
                break;
            }
        }

        // Slow down smoothly upon approaching the end of the path
        // This value will smoothly go from 1 to 0 as the agent approaches the last waypoint in the path.
        var speedFactor = reachedEndOfPath ? Mathf.Sqrt(distanceToWaypoint/nextWaypointDistance) : 1f;

        // Direction to the next waypoint
        // Normalize it so that it has a length of 1 world unit
        Vector3 dir = (path.vectorPath[currentWaypoint] - transform.position).normalized;
        // Multiply the direction by our desired speed to get a velocity
        Vector3 velocity = dir * speed * speedFactor;

        // Move the agent using the CharacterController component
        // Note that SimpleMove takes a velocity in meters/second, so we should not multiply by Time.deltaTime
        controller.SimpleMove(velocity);

        // If you are writing a 2D game you may want to remove the CharacterController and instead modify the position directly
        // transform.position += velocity * Time.deltaTime;
    }
}
```

- CollisionAvoidance.cs

```c#
using UnityEngine;
using System.Collections;
using Pathfinding.RVO;

public class SimpleRVOAI : MonoBehaviour {
    RVOController controller;

    // Use this for initialization
    void Awake () {
        controller = GetComponent<RVOController>();
    }

    // Update is called once per frame
    public void Update () {
        // Just some point far away
        var targetPoint = transform.position + transform.forward * 100;

        // Set the desired point to move towards using a desired speed of 10 and a max speed of 12
        controller.SetTarget(targetPoint, 10, 12);

        // Calculate how much to move during this frame
        // This information is based on movement commands from earlier frames
        // as local avoidance is calculated globally at regular intervals by the RVOSimulator component
        var delta = controller.CalculateMovementDelta(transform.position, Time.deltaTime);
        transform.position = transform.position + delta;
    }
}
```

- Modifier

```c#
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
```
