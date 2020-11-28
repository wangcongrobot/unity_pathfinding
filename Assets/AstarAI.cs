using UnityEngine;
using System;
using System.IO;
using System.Text;

// Note this line, if it is left out, the script won't know that the class 'Path' exists and it will throw compiler errors
// This line should always be present at the top of scripts which use pathfinding
using Pathfinding;

public class AstarAI : MonoBehaviour {
    public Transform targetPosition;

    private Seeker seeker;
    private CharacterController controller;

    public Pathfinding.Path path;

    public string file_path = "/home/cong/workspace/unity-motion-planning/unity/unity_pathfinding3/log.csv";

    public float speed = 2;

    public float nextWaypointDistance = 3;

    private int currentWaypoint = 0;

    public bool reachedEndOfPath;

    public PathHandler pathHandler;

    public PathNode pn;

    public void Start () {
        seeker = GetComponent<Seeker>();
        // If you are writing a 2D game you should remove this line
        // and use the alternative way to move sugggested further below.
        controller = GetComponent<CharacterController>();

        // Start a new path to the targetPosition, call the the OnPathComplete function
        // when the path has been calculated (which may take a few frames depending on the complexity)
        seeker.StartPath(transform.position, targetPosition.position, OnPathComplete);
    }

    public void OnPathComplete (Pathfinding.Path p) {
        Debug.Log("A path was calculated. Did it fail with an error? " + p.error);

        if (!p.error) {
            path = p;
            // Reset the waypoint counter so that we start to move towards the first point in the path
            currentWaypoint = 0;
        }
        // Debug.Log("Get a path: \n" + path);
        // Debug.Log("duration: " + path.heuristic);
        // Debug.Log("duration: " + path.vectorPath);
        // Debug.Log("duration: " + path.GetTotalLength());
        // Debug.Log("duration: " + path.GetTraversalCost);
        // Debug.Log("duration: " + path.duration);
        // break;
        // var stream = System.IO.File.CreateText(file_path);
        // var w = new StreamWriter(file_path);
        
        for (int i = 0; i < path.vectorPath.Count; i++)
        {
            string x = path.vectorPath[i][0].ToString();
            string y = path.vectorPath[i][1].ToString();
            string z = path.vectorPath[i][2].ToString();
            // File.WriteAllText(file_path, path.vectorPath[i][0].ToString());
            // System.IO.File.WriteAllText(file_path, path.vectorPath[i][0].ToString());
            // System.IO.File.AppendAllText(file_path, path.ToString());
            // Debug.Log("path x: " + i + " " + path.vectorPath[i][0].ToString());
            // Debug.Log("path y: " + i + " " + path.vectorPath[i][1].ToString());
            // Debug.Log("path z: " + i + " " + path.vectorPath[i][2].ToString());
            string csvRow = string.Format("{0},{1},{2},{3},\n",i, x, y, z);
            // stream.WriteLine(csvRow);
            // Debug.Log("Log to file" + i + " " + csvRow);
            System.IO.File.AppendAllText(file_path, csvRow);
            // System.IO.File.AppendAllLines(file_path, csvRow);
            // Debug.Log("Path info 1: " + path.path.Count);
            // Debug.Log("Path info 2: " + path.searchedNodes());
            // Debug.Log("Path info 3: " + path.GetTotalLength());
            // Debug.Log("Path info 4: " + path.GetTraversalCost());
            // Debug.Log("Path info 5: " + path.GetState());
            // Debug.Log("Path info 6: " + path.GetHTarget());
            // Debug.Log("Path info 7: " + path.duration);
            // Debug.Log("Path info 8: " + path.pathID);
            // Debug.Log("Path info 9: " + path.path);
            
            // if (pathHandler == null) FailWithError("Field pathHandler is not set. Please report this bug.");
            // PathNode nodeR = pathHandler.GetPathNode(path.path[8]);
            // Debug.Log("Path info 9: " + path.path[i].H);
            // Debug.Log("H: " + nodeR.H);

			// Mark nodes to enable special connection costs for start and end nodes
			// See GetConnectionSpecialCost
			// if (startNode != null) pathHandler.GetPathNode(startNode).flag2 = true;
			// if (endNode != null) pathHandler.GetPathNode(endNode).flag2 = true;

			// Zero out the properties on the start node
			// PathNode startRNode = pathHandler.GetPathNode(startNode);
			// startRNode.node = startNode;
			// startRNode.pathID = pathHandler.PathID;
			// startRNode.parent = null;
			// startRNode.cost = 0;
			// startRNode.G = GetTraversalCost(startNode);
			// startRNode.H = CalculateHScore(startNode);
        }

        pathHandler = new PathHandler(0,0);
        pathHandler.InitializeForPath(path);
        // Debug.Log("pathHandler: " + pathHandler.PathID);
        
        // pathHandler.InitializeForPath(path);        
        // System.IO.File.WriteAllText(file_path, csvRow);
        // Debug.Log("Path node count: " + path.path.Count);
        
        for (int j = 0; j < path.path.Count; j++)
        {
            // Debug.Log("Path Node index: " + j + " " + path.path[j].NodeIndex);
            // Debug.Log("Path Node position: " + j + " " + path.path[j].position);
            // Debug.Log("Path Node Penalty: " + j + " " + path.path[j].Penalty);
            // Debug.Log("Path Node position: " + j + " " + path.path[j].position);
            // Debug.Log("Path Node position: " + j + " " + path.path[j].position);

            // pn = pathHandler.GetPathNode(path.path[j].NodeIndex);
            // pathHandler.startNode

            // Debug.Log("pathHandler" + pathHandler.threadID);
            // Debug.Log("pathHandler" + pathHandler.totalThreadCount);
            // Debug.Log("pathHandler" + pathHandler.heap);
        }
        
    }

    public void Update () {
        if (path == null) {
            // We have no path to follow yet, so don't do anything
            return;
        }
        // Debug.Log("path: " + path);
        // return;

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

        // Debug.Log("path.vectorPath: " + currentWaypoint + path.vectorPath[currentWaypoint]);
        // Pathfinding.Serialization.AstarSerializer.SaveToFile(data_path, path.vectorPath);
        // FunnelModifier.WriteAllText(data_path, path.vectorPath);


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

        // If you are writing a 2D game you should remove the CharacterController code above and instead move the transform directly by uncommenting the next line
        // transform.position += velocity * Time.deltaTime;
    }

    //写CSV文件
    /// <summary>
    /// Write CSV File
    /// </summary>
    /// <param name="fileName"></param>
    /// <param name="id"></param>
    /// <param name="data"></param>
    public void WriteCVS(string fileName, int id, double data)
    {
        if (!File.Exists(fileName)) //当文件不存在时创建文件
        {
            //创建文件流(创建文件)
            FileStream fs = new FileStream(fileName, FileMode.Create, FileAccess.Write);
            //创建流写入对象，并绑定文件流
            StreamWriter sw = new StreamWriter(fs);
            //实例化字符串流
            StringBuilder sb = new StringBuilder();
            //将数据添加进字符串流中（如果数据标题有变更，修改此处）
            sb.Append("Num").Append(",").Append("Data").Append(",");
            //将字符串流数据写入文件
            sw.WriteLine(sb);
            //刷新文件流
            sw.Flush();
            sw.Close();
            fs.Close();
        }

        //将数据写入文件

        //实例化文件写入对象
        StreamWriter swd = new StreamWriter(fileName, true, Encoding.Default);
        StringBuilder sbd = new StringBuilder();
        //将需要保存的数据添加到字符串流中
        sbd.Append(id).Append(",").Append(data).Append(",");
        swd.WriteLine(sbd);
        swd.Flush();
        swd.Close();
    }

    /// <summary>
    /// Write CSV File
    /// </summary>
    /// <param name="fileName"></param>
    /// <param name="id"></param>
    /// <param name="data"></param>
    /// <param name="data1"></param>
    /// <param name="data2"></param>
    /// <param name="flag"></param>
    public void WriteCVS(string fileName, int id, double data, int data1, string data2, bool flag)
    {
        if (!File.Exists(fileName)) //当文件不存在时创建文件
        {
            //创建文件流(创建文件)
            FileStream fs = new FileStream(fileName, FileMode.Create, FileAccess.Write);
            //创建流写入对象，并绑定文件流
            StreamWriter sw = new StreamWriter(fs);
            //实例化字符串流
            StringBuilder sb = new StringBuilder();
            //将数据添加进字符串流中（如果数据标题有变更，修改此处）
            sb.Append("ID").Append(",").Append("Data").Append(",").Append("x").Append(",").Append("y").Append(",").Append("z").Append(",");
            //将字符串流数据写入文件
            sw.WriteLine(sb);
            //刷新文件流
            sw.Flush();
            sw.Close();
            fs.Close();
        }
        //将数据写入文件
        //实例化文件写入对象
        StreamWriter swd = new StreamWriter(fileName, true, Encoding.Default);
        StringBuilder sbd = new StringBuilder();
        //将需要保存的数据添加到字符串流中
        sbd.Append(id).Append(",").Append(data).Append(",").Append(data1).Append(",").Append(data2).Append(",").Append(flag).Append(",");
        swd.WriteLine(sbd);
        swd.Flush();
        swd.Close();
    }
}