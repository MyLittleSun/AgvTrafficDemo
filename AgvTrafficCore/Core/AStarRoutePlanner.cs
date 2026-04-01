using AGV.Graph.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace AgvTrafficCore.Core
{
    /// <summary>
    /// 使用A*算法进行路径规划的类。
    /// </summary>
    internal class AStarRoutePlanner
    {
        public static List<IEdge> FindRoute(IGraph<IVertex, IEdge> graph, IVertex start, IVertex goal)
        {
            ArgumentNullException.ThrowIfNull(graph);
            ArgumentNullException.ThrowIfNull(start);
            ArgumentNullException.ThrowIfNull(goal);

            if (start == goal) return [];

            var openlist = new PriorityQueue<AStarNode, decimal>();
            var gScore = new Dictionary<IVertex, decimal>();
            var closeSet = new HashSet<IVertex>();

            openlist.Enqueue(new AStarNode(start), 0m);
            gScore[start] = 0m;

            while (openlist.Count > 0)
            {
                var currNode = openlist.Dequeue();
                if (currNode.Vertex == goal)
                {
                    return ReconstructEdges(currNode);
                }
                if (closeSet.Contains(currNode.Vertex))
                {
                    continue;
                }
                closeSet.Add(currNode.Vertex);

                foreach (var edge in graph.OutgoingEdges(currNode.Vertex))
                {
                    var neighbor = edge.Dst;
                    // 跳过不可用的路线
                    if (edge.Weight <= 0 || closeSet.Contains(neighbor))
                    {
                        continue;
                    }

                    var neighborNode = new AStarNode(neighbor, edge, currNode);
                    decimal fScore = neighborNode.G + GetDistance(currNode.Vertex, neighbor);

                    if (!gScore.TryGetValue(neighbor, out var oldG) || fScore < oldG)
                    {
                        gScore[neighbor] = fScore;
                        openlist.Enqueue(neighborNode, fScore);
                    }
                }
            }

            throw new InvalidOperationException($"A* failed to find a path from '{start}' to '{goal}'.");
        }

        private static decimal GetDistance(IVertex src, IVertex dst)
        {
            return Math.Abs(dst.X - src.X) + Math.Abs(dst.Y - src.Y);
        }

        private static List<IEdge> ReconstructEdges(AStarNode goal)
        {
            var edges = new List<IEdge>();

            while (goal.Edge != null)
            {
                edges.Add(goal.Edge);

                if (goal.Parent == null)
                {
                    break;
                }
                goal = goal.Parent;
            }

            edges.Reverse();
            return edges;
        }
    }

    /// <summary>
    /// 规划过程中使用的节点类，包含当前顶点、通过哪个边到达、父节点以及从起点到当前节点的实际代价G。
    /// </summary>
    internal class AStarNode
    {
        public IVertex Vertex { get; }
        public IEdge? Edge { get; }
        public AStarNode? Parent { get; }
        public decimal G { get; }

        public AStarNode(IVertex vertex, IEdge? edge = null, AStarNode? parent = null)
        {
            Vertex = vertex;
            Edge = edge;
            Parent = parent;
            G = parent == null ? 0 : parent.G;
            if (edge != null)
            {
                G += edge.Distance / edge.VelocityMax;
            }
        }
    }
}
