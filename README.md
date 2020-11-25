# Unity Pathfinding Project

## Unity

unity version: 2018.4.28f1

## project

Assets/unity_pathfinding/ExampleScenes/unity_pathfinding

Assets/unity_pathfinding/Core/AstarPath.cs

Assets/unity_pathfinding/Core/AI/Seeker.cs


我们将使用Unity内置组件[CharacterController](https://docs.unity3d.com/ScriptReference/CharacterController.html)移动agent。因此，将一个CharacterController附加到AI GameObject。

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

- 
```c#
		public void UpdateG (Path path) {
#if ASTAR_NO_TRAVERSAL_COST
			g = parent.g + cost;
#else
			g = parent.g + cost + path.GetTraversalCost(node);
#endif
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

```

- 
```c#

```

- 
```c#

```

- 
```c#

```