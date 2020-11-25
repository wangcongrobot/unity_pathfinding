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

## Testing

- Heuristic: Manhattan
```
Path Completed : Computation Time 26.398 ms Searched Nodes 1090 Path Length 89
End Node
	G: 247972
	H: 2500
	F: 250472
	Point: (200.0, 4.9, 350.0)
	Graph: 0
Start Node
	Point: (133.4, 0.0, 131.3)
	Graph: 0
Path Number 1 (unique id)
```
![](videos/unity_pathfinding_Manhattan.mp4)


- Heuristic: Diagonal Manhattan
```
Path Completed : Computation Time 26.486 ms Searched Nodes 1411 Path Length 89
End Node
	G: 247972
	H: 1750
	F: 249722
	Point: (200.0, 4.9, 350.0)
	Graph: 0
Start Node
	Point: (133.4, 0.0, 131.3)
	Graph: 0
Path Number 1 (unique id)
```
![](videos/unity_pathfinding_Diagonal_Manhattan.mp4)

- Heuristic: Euclidean
```
Path Completed : Computation Time 30.688 ms Searched Nodes 2432 Path Length 89
End Node
	G: 247972
	H: 1768
	F: 249740
	Point: (200.0, 4.9, 350.0)
	Graph: 0
Start Node
	Point: (133.4, 0.0, 131.3)
	Graph: 0
Path Number 1 (unique id)
```
![](videos/unity_pathfinding_Euclidean.mp4)

- Heuristic: None
```
Path Completed : Computation Time 61.724 ms Searched Nodes 18247 Path Length 89
End Node
	G: 247972
	H: 0
	F: 247972
	Point: (200.0, 4.9, 350.0)
	Graph: 0
Start Node
	Point: (133.4, 0.0, 131.3)
	Graph: 0
Path Number 1 (unique id)
```
![](videos/unity_pathfinding_None.mp4)

- Multi-Agents

![](videos/unity_pathfinding_Multi_agents.mp4)

## A* Algorithm

- CalculateHScore
```c#

		/// <summary>
		/// Estimated cost from the specified node to the target.
		/// See: https://en.wikipedia.org/wiki/A*_search_algorithm
		/// </summary>
		internal uint CalculateHScore (GraphNode node) {
			uint h;

			switch (heuristic) {
			case Heuristic.Euclidean:
				h = (uint)(((GetHTarget() - node.position).costMagnitude)*heuristicScale);
				// Inlining this check and the return
				// for each case saves an extra jump.
				// This code is pretty hot
				if (hTargetNode != null) {
					h = System.Math.Max(h, AstarPath.active.euclideanEmbedding.GetHeuristic(node.NodeIndex, hTargetNode.NodeIndex));
				}
				return h;
			case Heuristic.Manhattan:
				Int3 p2 = node.position;
				h = (uint)((System.Math.Abs(hTarget.x-p2.x) + System.Math.Abs(hTarget.y-p2.y) + System.Math.Abs(hTarget.z-p2.z))*heuristicScale);
				if (hTargetNode != null) {
					h = System.Math.Max(h, AstarPath.active.euclideanEmbedding.GetHeuristic(node.NodeIndex, hTargetNode.NodeIndex));
				}
				return h;
			case Heuristic.DiagonalManhattan:
				Int3 p = GetHTarget() - node.position;
				p.x = System.Math.Abs(p.x);
				p.y = System.Math.Abs(p.y);
				p.z = System.Math.Abs(p.z);
				int diag = System.Math.Min(p.x, p.z);
				int diag2 = System.Math.Max(p.x, p.z);
				h = (uint)((((14*diag)/10) + (diag2-diag) + p.y) * heuristicScale);
				if (hTargetNode != null) {
					h = System.Math.Max(h, AstarPath.active.euclideanEmbedding.GetHeuristic(node.NodeIndex, hTargetNode.NodeIndex));
				}
				return h;
			}
			return 0U;
		}
```

- 
```c#

	/// <summary>How to visualize the graphs in the editor</summary>
	public enum GraphDebugMode {
		/// <summary>Draw the graphs with a single solid color</summary>
		SolidColor,
		/// <summary>
		/// Use the G score of the last calculated paths to color the graph.
		/// The G score is the cost from the start node to the given node.
		/// See: https://en.wikipedia.org/wiki/A*_search_algorithm
		/// </summary>
		G,
		/// <summary>
		/// Use the H score (heuristic) of the last calculated paths to color the graph.
		/// The H score is the estimated cost from the current node to the target.
		/// See: https://en.wikipedia.org/wiki/A*_search_algorithm
		/// </summary>
		H,
		/// <summary>
		/// Use the F score of the last calculated paths to color the graph.
		/// The F score is the G score + the H score, or in other words the estimated cost total cost of the path.
		/// See: https://en.wikipedia.org/wiki/A*_search_algorithm
		/// </summary>
		F,
		/// <summary>
		/// Use the penalty of each node to color the graph.
		/// This does not show penalties added by tags.
		/// See: graph-updates (view in online documentation for working links)
		/// See: <see cref="Pathfinding.GraphNode.Penalty"/>
		/// </summary>
		Penalty,
		/// <summary>
		/// Visualize the connected components of the graph.
		/// A node with a given color can reach any other node with the same color.
		///
		/// See: <see cref="Pathfinding.HierarchicalGraph"/>
		/// See: https://en.wikipedia.org/wiki/Connected_component_(graph_theory)
		/// </summary>
		Areas,
		/// <summary>
		/// Use the tag of each node to color the graph.
		/// See: tags (view in online documentation for working links)
		/// See: <see cref="Pathfinding.GraphNode.Tag"/>
		/// </summary>
		Tags,
		/// <summary>
		/// Visualize the hierarchical graph structure of the graph.
		/// This is mostly for internal use.
		/// See: <see cref="Pathfinding.HierarchicalGraph"/>
		/// </summary>
		HierarchicalNode,
	}
```

- PathHandler
```c#
		/// <summary>Backing field for the G score</summary>
		private uint g;

		/// <summary>Backing field for the H score</summary>
		private uint h;

		/// <summary>G score, cost to get to this node</summary>
		public uint G { get { return g; } set { g = value; } }

		/// <summary>H score, estimated cost to get to to the target</summary>
		public uint H { get { return h; } set { h = value; } }

		/// <summary>F score. H score + G score</summary>
		public uint F { get { return g+h; } }

		public void UpdateG (Path path) {
#if ASTAR_NO_TRAVERSAL_COST
			g = parent.g + cost;
#else
			g = parent.g + cost + path.GetTraversalCost(node);
#endif
		}
	}
```

- 
```c#
		protected override void Initialize () {
			// Mark nodes to enable special connection costs for start and end nodes
			// See GetConnectionSpecialCost
			if (startNode != null) pathHandler.GetPathNode(startNode).flag2 = true;
			if (endNode != null) pathHandler.GetPathNode(endNode).flag2 = true;

			// Zero out the properties on the start node
			PathNode startRNode = pathHandler.GetPathNode(startNode);
			startRNode.node = startNode;
			startRNode.pathID = pathHandler.PathID;
			startRNode.parent = null;
			startRNode.cost = 0;
			startRNode.G = GetTraversalCost(startNode);
			startRNode.H = CalculateHScore(startNode);

			// Check if the start node is the target and complete the path if that is the case
			CompletePathIfStartIsValidTarget();
			if (CompleteState == PathCompleteState.Complete) return;

			// Open the start node and puts its neighbours in the open list
			startNode.Open(this, startRNode, pathHandler);

			searchedNodes++;

			partialBestTarget = startRNode;

			// Any nodes left to search?
			if (pathHandler.heap.isEmpty) {
				if (calculatePartial) {
					CompleteState = PathCompleteState.Partial;
					Trace(partialBestTarget);
				} else {
					FailWithError("No open points, the start node didn't open any nodes");
				}
				return;
			}

			// Pop the first node off the open list
			currentR = pathHandler.heap.Remove();
		}
```

- 
```c#
		/// <summary>
		/// Writes text shared for all overrides of DebugString to the string builder.
		/// See: DebugString
		/// </summary>
		protected void DebugStringPrefix (PathLog logMode, System.Text.StringBuilder text) {
			text.Append(error ? "Path Failed : " : "Path Completed : ");
			text.Append("Computation Time ");
			text.Append(duration.ToString(logMode == PathLog.Heavy ? "0.000 ms " : "0.00 ms "));

			text.Append("Searched Nodes ").Append(searchedNodes);

			if (!error) {
				text.Append(" Path Length ");
				text.Append(path == null ? "Null" : path.Count.ToString());
			}
		}
```

- 
```c#
		/// <summary>Returns a debug string for this path.</summary>
		internal override string DebugString (PathLog logMode) {
			if (logMode == PathLog.None || (!error && logMode == PathLog.OnlyErrors)) {
				return "";
			}

			var text = new System.Text.StringBuilder();

			DebugStringPrefix(logMode, text);

			if (!error && logMode == PathLog.Heavy) {
				if (hasEndPoint && endNode != null) {
					PathNode nodeR = pathHandler.GetPathNode(endNode);
					text.Append("\nEnd Node\n	G: ");
					text.Append(nodeR.G);
					text.Append("\n	H: ");
					text.Append(nodeR.H);
					text.Append("\n	F: ");
					text.Append(nodeR.F);
					text.Append("\n	Point: ");
					text.Append(((Vector3)endPoint).ToString());
					text.Append("\n	Graph: ");
					text.Append(endNode.GraphIndex);
				}

				text.Append("\nStart Node");
				text.Append("\n	Point: ");
				text.Append(((Vector3)startPoint).ToString());
				text.Append("\n	Graph: ");
				if (startNode != null) text.Append(startNode.GraphIndex);
				else text.Append("< null startNode >");
			}

			DebugStringSuffix(logMode, text);

			return text.ToString();
		}
```

- 
```c#
	/// <summary>
	/// Basic path, finds the shortest path from A to B.
	/// \ingroup paths
	/// This is the most basic path object it will try to find the shortest path between two points.\n
	/// Many other path types inherit from this type.
	/// See: Seeker.StartPath
	/// See: calling-pathfinding (view in online documentation for working links)
	/// See: getstarted (view in online documentation for working links)
	/// </summary>
	public class ABPath : Path {
		/// <summary>Start node of the path</summary>
		public GraphNode startNode;

		/// <summary>End node of the path</summary>
		public GraphNode endNode;

		/// <summary>Start Point exactly as in the path request</summary>
		public Vector3 originalStartPoint;

		/// <summary>End Point exactly as in the path request</summary>
		public Vector3 originalEndPoint;

		/// <summary>
		/// Start point of the path.
		/// This is the closest point on the <see cref="startNode"/> to <see cref="originalStartPoint"/>
		/// </summary>
		public Vector3 startPoint;

		/// <summary>
		/// End point of the path.
		/// This is the closest point on the <see cref="endNode"/> to <see cref="originalEndPoint"/>
		/// </summary>
		public Vector3 endPoint;

		/// <summary>
		/// Determines if a search for an end node should be done.
		/// Set by different path types.
		/// \since Added in 3.0.8.3
		/// </summary>
		protected virtual bool hasEndPoint {
			get {
				return true;
			}
		}

		public Int3 startIntPoint; /// <summary>< Start point in integer coordinates</summary>

		/// <summary>
		/// Calculate partial path if the target node cannot be reached.
		/// If the target node cannot be reached, the node which was closest (given by heuristic) will be chosen as target node
		/// and a partial path will be returned.
		/// This only works if a heuristic is used (which is the default).
		/// If a partial path is found, CompleteState is set to Partial.
		/// Note: It is not required by other path types to respect this setting
		///
		/// Warning: This feature is currently a work in progress and may not work in the current version
		/// </summary>
		public bool calculatePartial;

		/// <summary>
		/// Current best target for the partial path.
		/// This is the node with the lowest H score.
		/// Warning: This feature is currently a work in progress and may not work in the current version
		/// </summary>
		protected PathNode partialBestTarget;

		/// <summary>Saved original costs for the end node. See: ResetCosts</summary>
		protected int[] endNodeCosts;

#if !ASTAR_NO_GRID_GRAPH
		/// <summary>Used in EndPointGridGraphSpecialCase</summary>
		GridNode gridSpecialCaseNode;
#endif
```
