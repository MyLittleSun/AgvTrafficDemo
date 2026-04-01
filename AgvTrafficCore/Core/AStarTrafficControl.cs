using AGV.Graph.Interfaces;
using AGV.TrafficControl2.Core;
using AGV.TrafficControl2.Core.Geometry;
using AGV.TrafficControl2.Plugins.DeadLockControl;
using AgvTrafficCore.Core;
using log4net;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Reflection;
using System.Runtime.CompilerServices;

namespace AGV.TrafficControl2
{
    /// <summary>
    /// 基于 A* 的简化管制实现
    /// </summary>
    public sealed class AStarTrafficControl : ITrafficControl
    {
        /// <summary>
        /// 当前管制系统使用的拓扑图，包含所有站点（Vertex）及路径（Edge）关系
        /// </summary>
        public IGraph<IVertex, IEdge> Graph { get; private set; }
        public ILog PlaningLog { get; private set; }
        public ILog ArrivalLog { get; private set; }
        /// <summary>
        /// 车辆以指定角度在站点上的关联路线(会发生碰撞的路线)，例如：当车辆在站点 A 以弧度 1.57 时，可能与路径 B-C 和 D-E 发生碰撞，则 RelationPathsOfStation[A][1.57] 包含 B-C 和 D-E。
        /// </summary>
        private Dictionary<IVertex, Dictionary<decimal, List<IEdge>>>? RelationPathsOfStation { get; set; }
        /// <summary>
        /// 车辆占用某段路径时，可能发生碰撞的其他路径列表，例如：当车辆占用路径 A-B 时，可能与路径 C-D 发生碰撞，则 RelationPathsOfPath[A-B] 包含 C-D。
        /// </summary>
        private Dictionary<IEdge, List<IEdge>>? RelationPathsOfPath { get; set; }
        /// <summary>
        /// 占用路线的最大长度
        /// </summary>
        private decimal MaxPassLength { get; set; }

        /// <summary>
        /// 保存所有小车的管制信息(规划的完整路线以及占用路线)
        /// </summary>
        private readonly Dictionary<string, ControlCar> _cars = new Dictionary<string, ControlCar>();


        /// <summary>
        /// 程序启动时初始化管制模块及创建对应的车辆信息
        /// </summary>
        /// <param name="parameter"></param>
        public void Initialize(TrafficInitializeParameter parameter)
        {
            Graph = parameter.Graph;
            PlaningLog = parameter.PlaningLog;
            ArrivalLog = parameter.ArrivalLog;
            RelationPathsOfStation = parameter.RelationPathsOfStation;
            RelationPathsOfPath = parameter.RelationPathsOfPath;
            MaxPassLength = parameter.MaxPassLength;

            // 添加所有的车辆信息
            _cars.Clear();
            foreach(var vehicle in parameter.Cars)
            {
                _cars.Add(vehicle.Key, new ControlCar(vehicle.Key, vehicle.Value));
            }
        }

        /// <summary>
        /// 重置小车当前站点，程序启动时或者人工手动拉车后会调用此方法，新的站点不在规划路线中。
        /// </summary>
        public void InitializeCar(string carID, IVertex vertex, decimal theta)
        {
            var car = GetControlCar(carID);
            ClearControl(carID);
            car.CurrentStation = vertex;

            // TODO: 可进行管制路线更新更新等操作
        }


        /// <summary>
        /// 规划路线
        /// </summary>
        /// <param name="carID">待规划车辆</param>
        /// <param name="targetList">目标点列表(第一个站点为实际目标点，其他站点为后续可能前往的目标点可能会变)</param>
        /// <param name="updateTargetPoint">是否在路线规划完成后立刻更新占用路线</param>
        /// <returns>返回为前往目标点集的路线集合</returns>
        /// 例如： targetList = [5, 8, 10]，则可能的返回值为 [[1, 2, 3, 5], [5, 6, 7, 8], [8, 9, 10]] 
        public List<IEdge>[] PlanRoute(string carID, List<IVertex> targetList, bool updateTargetPoint)
        {
            if (targetList.Count == 0) return []; 

            // 用无管制规划直接生成路径；updateTargetPoint 在示例中不做额外逻辑。
            var routes = PlanRouteWithoutControlInternal(carID, targetList);
            if (updateTargetPoint)
            {
                UpdateTrafficPoint(GetControlCar(carID));
            }
            return routes;
        }

        /// <summary>
        /// 同上
        /// </summary>
        public List<IEdge>[] PlanRoute(string carID, PlanningParameter parameter)
        {
            ArgumentNullException.ThrowIfNull(parameter);
            parameter.CheckParamter();

            var targets = parameter.Goals.Select(p => p.Key).ToList();
            return PlanRouteWithoutControlInternal(carID, targets, needWholeRoute: false, ignoreAngleConstraintOfStart: parameter.IgnoreAngleConstraintOfStart);
        }

        /// <summary>
        /// 同上，区别在于以给定起点进行规划
        /// </summary>
        public List<IEdge>[] PlanRouteFromVertex(string carID, IVertex curVertex, decimal theta, PlanningParameter parameter)
        {
            ArgumentNullException.ThrowIfNull(curVertex);
            ArgumentNullException.ThrowIfNull(parameter);
            parameter.CheckParamter();


            // 此处为示例阶段的简化处理，直接将小车当前站点设置为 curVertex
            GetControlCar(carID).CurrentStation = curVertex;

            return PlanRoute(carID, parameter);
        }

        /// <summary>
        /// 直接以最短路线输出
        /// </summary>
        public List<IEdge>[] PlanRouteWithoutControl(string carID, List<IVertex> list, bool needWholeRoute = false, bool ignoreAngleConstraintOfStart = true)
        {
            if (list == null) throw new ArgumentNullException(nameof(list));
            if (list.Count == 0) throw new ArgumentException("list is empty", nameof(list));

            return PlanRouteWithoutControlInternal(carID, list, needWholeRoute, ignoreAngleConstraintOfStart);
        }

        /// <summary>
        /// 小车到站处理，释放走过的路线，占用新的路线
        /// </summary>
        public CarInfo ArrivalStation(string carID, IVertex vertex)
        {
            var car = GetControlCar(carID);
            car.CurrentStation = vertex;

            foreach (var path in car.OccupiedPath.ToList())
            {
                if (path.Src == vertex) break;

                car.OccupiedPath.Remove(path);
                //ArrivalLog.Info($"\t{carID} release occupied path {path}");
            }
            UpdateTrafficPoint(car);

            return GetCarInfo(car.ID);
        }

        /// <summary>
        /// 小车运行时会提前占用未来几段路线，此函数用于释放这些占用路线
        /// </summary>
        public void RecoverOccupiedPath(string carID, IVertex vertex = null)
        {
        }

        /// <summary>
        /// 根据小车当前站点的轮廓和朝向，获取可能相关（例如可能发生碰撞）的其他车辆列表。
        /// </summary>
        public List<string> RelatedCars(IVertex vertex, VehicleModel model, decimal goalAngle)
        {
            // 示例阶段：不做实际轮廓/角度关联
            return new List<string>();
        }

        /// <summary>
        /// 清除小车的管制状态，例如占用路径、当前站点等，通常在小车被人工干预（例如拉车）后调用。
        /// </summary>
        public void ClearControl(string carID)
        {
            var car = GetControlCar(carID);

            car.OccupiedPath.Clear();
            car.RemainRoute.Clear();
            car.ReverseRoute.Clear();
            // TODO: 根据实际需求清理管制状态，例如清空占用路径等
        }

        /// <summary>
        /// 获取关联路径计算器接口，用于计算路线之间是否发生碰撞。
        /// </summary>
        /// <returns></returns>
        public IPublicRelationPathCalculator GetRelationCalculator()
        {
            throw new NotSupportedException("Relation calculator is not implemented in this A* sample.");
        }

        public CarInfo GetCarInfo(string carID)
        {
            return new CarInfo(GetControlCar(carID));
        }

        public ControlCar GetControlCar(string carID)
        {
            if (_cars.TryGetValue(carID, out var car))
            {
                return car;
            }
            throw new KeyNotFoundException($"Unknown carID: {carID}");
        }

        public PathInfo GetPathInfo(string pathID)
        {
            throw new NotSupportedException("PathInfo is not implemented in this A* sample.");
        }

        public List<string> GetAllCarsInRegion(Rectangle bounds)
        {
            return [];
        }

        public List<string> GetAllCarsInRegion(List<Point> Points)
        {
            return []; 
        }

        public bool IsEasyCollision(string carId, string otherCarId)
        {
            return false;
        }

        public void TryResolveStationConflict(string carId, out List<string> resolvedConflict)
        {
            resolvedConflict = new List<string>();
        }

        /// <summary>
        /// 规划路线
        /// </summary>
        /// <param name="carID">待规划车辆</param>
        /// <param name="targets">目标点列表(第一个站点为实际目标点，其他站点为后续可能前往的目标点可能会变)</param>
        /// <param name="updateTargetPoint">是否在路线规划完成后立刻更新占用路线</param>
        /// <param name="targets">目标点列表</param>
        /// <param name="needWholeRoute">返回实际路线时(返回值indexp[0]对应的路线)是否需要包含占用路线</param>
        /// <param name="ignoreAngleConstraintOfStart">是否需要忽略起始车辆角度(不能规划定点旋转的路线,即上段路线终点角度和下段路线起始角度差不能超过90度)</param>
        /// <returns>返回为前往目标点集的路线集合</returns>
        /// 例如： targetList = [5, 8, 10]，则可能的返回值为 [[1, 2, 3, 5], [5, 6, 7, 8], [8, 9, 10]] 
        private List<IEdge>[] PlanRouteWithoutControlInternal(string carID, List<IVertex> targets, bool needWholeRoute = false, bool ignoreAngleConstraintOfStart = true)
        {
            var car = GetControlCar(carID);
            // 以占用路线终点或当前站点作为起点，依次规划到每个目标点的路径段
            var start = car.OccupiedPath.LastOrDefault()?.Edge.Dst ?? car.CurrentStation;
            var routes = new List<IEdge>[targets.Count];

            for (int i = 0; i < targets.Count; i++)
            {
                var goal = targets[i];
                var segmentEdges = AStarRoutePlanner.FindRoute(Graph, start, goal);
                routes[i] = segmentEdges;
                start = goal;
            }
            if (routes.Length > 0)
            {
                // 清理之前的路径状态
                ClearControl(carID);

                // 将规划的路径段转换为 ControlPath 并添加到小车的管制状态中
                ControlPath? parent = null;
                for (int i = 0; i < routes.Length; i++)
                {
                    foreach (var edge in routes[i])
                    {
                        var controlPath = parent == null ? new ControlPath(edge, car, car.DefaultVehicleModel) 
                            : new ControlPath(parent, edge, car.DefaultVehicleModel);

                        // 下标0的对应的是实际路线，其余为预约路线
                        (i == 0 ? car.RemainRoute : car.ReverseRoute).Add(controlPath);

                        parent = controlPath;
                    }
                }

                // 如果需要返回全路径，则将占用路径添加到返回结果前面
                if (needWholeRoute)
                {
                    var occupiedEdges = car.OccupiedPath.Select(p => p.Edge).ToList();
                    if (occupiedEdges.Count > 0)
                    {
                        routes[0] = occupiedEdges.Concat(routes[0]).ToList();
                    }
                }
            }
            return routes;
        }

        /// <summary>
        /// 更新小车的占用路线。直接占用规划路线的前几段路径，直到占用长度达到 MaxPassLength 或者发生循环占用（即占用的路径中存在起点与当前占用路径终点相同的路径）
        /// </summary>
        private void UpdateTrafficPoint(ControlCar car)
        {
            static decimal adjustDistance(decimal dis) => dis > 4000 ? 4000 : dis;

            List<ControlPath> nextOccupied = new List<ControlPath>();
            decimal currentLength = car.OccupiedPath.Sum(p => adjustDistance(p.Distance));
            foreach (var path in car.RemainRoute)
            {
                decimal len = nextOccupied.Sum(t => adjustDistance(t.Distance));
                // 超过最大占用长度，或者路径存在循环，跳出占用
                if (currentLength + len >= MaxPassLength || car.OccupiedPath.Any(p => p.Src == path.Dst) || nextOccupied.Any(p => p.Src == path.Dst))
                {
                    break;
                }
                nextOccupied.Add(path);
            }
            foreach (var path in nextOccupied)
            {
                path.PathStatus = ControlPath.Status.Using;
                car.OccupiedPath.Add(path);
                car.RemainRoute.Remove(path);
            }

            //ArrivalLog.Info($"\t{car.ID} target station is {car.OccupiedPath.LastOrDefault()?.Station ?? car.CurrentStation}, " +
            //    $"occupied path is {car.CurrentStation}{string.Join("", car.OccupiedPath.Where(p => p.Edge != null).Select(p => "->" + p.Dst))}");
        }
    }
}

