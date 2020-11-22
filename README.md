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

![](videos/unity_pathfinding_Manhattan.mp4)


- Heuristic: Diagonal Manhattan

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

![](videos/unity_pathfinding_Diagonal_Manhattan.mp4)

- Heuristic: Euclidean

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

![](videos/unity_pathfinding_Euclidean.mp4)

- Heuristic: None

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

![](videos/unity_pathfinding_None.mp4)

- Multi-Agents

![](videos/unity_pathfinding_Multi_agents.mp4)