using AGV.Common.Diagnostics;
using AGV.Common.Helper;
using AGV.DTO;
using AGV.EF;
using AGV.Graph.Interfaces;
using AGV.TrafficControl2;
using AGV.TrafficControl2.Core;
using log4net;
using Microsoft.EntityFrameworkCore;
using System.ComponentModel.DataAnnotations;
using System.Xml.XPath;

namespace AgvTrafficCore
{
    internal class Program
    {
        static void Main(string[] args)
        {
            var graph = new SimpledirectedGraph();
            var carMap = new Dictionary<string, VehicleModel>();
            #region 加载数据库数据构建图
            using var dbContext = new MySqlContext();
            var stations = dbContext.station.ToList();
            var paths = dbContext.path.ToList();
            var cars = dbContext.agv_car.ToList();

            var stationMap = new Dictionary<string, StationDTO>();
            foreach (var st in stations)
            {
                var stationDto = new StationDTO()
                {
                    Identity = st.station_rfid,
                    X = st.space_x,
                    Y = st.space_y
                };
                stationMap[stationDto.Identity] = stationDto;
                graph.AddVertex(stationDto);
            }
            foreach (var pt in paths)
            {
                var pathDto = new PathDTO()
                {
                    Identity = pt.from_station_rfid + pt.to_station_rfid,
                    FromStation = stationMap[pt.from_station_rfid],
                    ToStation = stationMap[pt.to_station_rfid],
                    Distance = pt.distance * 1000,
                    VelocityMax = pt.velocity_max ?? 800,
                    Weight = 1
                };
                graph.AddEdge(pathDto);
            }
            foreach (var c in cars)
            {
                carMap[c.name] = new VehicleModel(c.can_rotate, c.car_type, c.length, c.width, c.reference_point);
            }
            // 数据库数据加载完成
            Console.WriteLine("Data loaded and processed successfully");
            #endregion

            var parameter = new TrafficInitializeParameter()
            {
                Graph = graph,
                Cars = carMap,
                //ArrivalLog = LogInstance.TrafficCtlLog,
                //PlaningLog = LogInstance.PlanningPathLog,
            };
            var trafficControl = new AStarTrafficControl();
            // 初始化交通控制模块
            trafficControl.Initialize(parameter);
            Console.WriteLine("Traffic control module initialized successfully");

            // 构建车辆数据(后期可通过数据库中task拿任务)
            var car = new
            {
                name = carMap.First().Key,
                station = graph.GetVertex("00209"),
                theta = -1.57m
            };
            // 车辆上线，初始化站点角度等
            trafficControl.InitializeCar(car.name, car.station, car.theta);
            Console.WriteLine($"{car.name} joined the system. Current node:{car.station}, theta:{car.theta}");

            // 规划路径，目标点可以是多个，模块会返回多条路径分别对应不同的目标点
            var goals = new List<IVertex>() { graph.GetVertex("00225"), graph.GetVertex("00347") };
            var routes = trafficControl.PlanRoute(car.name, goals, true);
            foreach (var route in routes)
            {
                Console.WriteLine($"{car.name} planned route to dest {route.Last().Dst.Identity}: {route.First().Src.Identity}->{string.Join("->", route.Select(p => p.Dst.Identity))}");
            }

            // 获取车辆信息，包含当前占用的路径和站点等
            var carInfo = trafficControl.GetCarInfo(car.name);
            // 模拟车辆行驶过程，依次到达路径上的站点
            while (carInfo.OccupiedPath.Count > 0)
            {
                Console.WriteLine($"{car.name} at {carInfo.CurrStation?.Identity} occupied path: {carInfo.CurrStation?.Identity}->{string.Join("->", carInfo.OccupiedPath.Select(p => p.Dst.Identity))}");
                var curr = carInfo.OccupiedPath.First().Dst;
                carInfo = trafficControl.ArrivalStation(car.name, curr);
                Thread.Sleep(1000);
            }
            Console.WriteLine($"{car.name} reached the task destination");
            Console.ReadKey();
        }
    }
}
