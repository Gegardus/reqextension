/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package reqextension;

import java.util.List;
import java.util.Set;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.HashMap;
import java.util.HashSet;
import java.util.function.Consumer;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Collections;
import java.util.Comparator;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */

public class MapGraph {		
	private Map<GeographicPoint, MapNode> graph;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph() {		
		graph = new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {		
		return graph.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {		
		Set<GeographicPoint> points = new HashSet<>();
		points.addAll(graph.keySet());
		return points;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {		
		int num = 0;
		for(Map.Entry<GeographicPoint, MapNode> edge : graph.entrySet())
			num += edge.getValue().getEdge().size();
			
		return num;
	}
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		if(location==null || graph.containsKey(location))return false;
		MapNode tmp = new MapNode(location);
		graph.put(location, tmp);
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length) throws IllegalArgumentException {
		
		try {
			if(!graph.containsKey(from) || !graph.containsKey(to) || length<=0)
				throw new IllegalArgumentException();
		}
		catch(IllegalArgumentException e) {
			
		}
		
		MapNode x = graph.get(from);
		MapNode y = graph.get(to);
		
		MapEdges xy = new MapEdges(x, y, roadName, roadType, length);
		
		x.addEdge(xy);
		
	}
	
    private List<GeographicPoint> path(MapNode s, MapNode edges, HashMap<MapNode, MapNode> node) {
		List<GeographicPoint> points = new LinkedList<>();
		MapNode tmp = edges;
		
		while(tmp != null && !tmp.equals(s)) {
			points.add(tmp.getLocation());
			tmp = node.get(tmp);
		}
		
		points.add(s.getLocation());
		Collections.reverse(points);
		return points;
	}
	
    /** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 *   
	 *  
	 */
	
	//method to initiate the distance as infinite
	public void initiate() {
		for(Map.Entry<GeographicPoint, MapNode> e : graph.entrySet()) {
			MapNode tmp=e.getValue();
			tmp.setWeight((double)Integer.MAX_VALUE);
			tmp.setValue();
			tmp.setTimeWeight((double)Integer.MAX_VALUE);
		}
	}
	
	
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
	
		initiate();
		if(start==null || goal==null) {
			System.out.println("Cannot find path:Information glitch");
			return null;
		}
		
		if(start==goal) {
			System.out.println("Start and end nodes are the same");
			return null;
		}
		
		MapNode startNode = graph.get(start);
		MapNode endNode = graph.get(goal);
		
		//for finding out the smallest
		PriorityQueue<MapNode> pq = new PriorityQueue<MapNode>(new DijkstraComparator());
		
		//To maintain visited nodes
		HashSet<GeographicPoint> vis = new HashSet<>();
		
		//To have parent array
		HashMap<MapNode, MapNode> par = new HashMap<>();
		
		//to check whether we have found solution or not.
		int count = 0;
		
		startNode.setWeight(0.0);
		pq.offer(startNode);
		
		while(!pq.isEmpty()) {
			
			MapNode tmp = pq.poll();
			double cost = tmp.getWeight();
			
			if(!vis.contains(tmp.getLocation())) {
				vis.add(tmp.getLocation());
				
				nodeSearched.accept(tmp.getLocation());
				++count;
			
				if(goal.equals(tmp.getLocation())) {
					System.out.println("Djikstra : " + count);
					return path(startNode,endNode,par);
				}
				
				for(MapEdges e:tmp.getEdge()) {
					MapNode gp=e.getOppositeEnd(tmp);
					if(!vis.contains(gp.getLocation())) {
						double check=cost+e.getDistance();
						if(gp.getWeight()>check)
						{
							gp.setWeight(check);
							par.put(gp,tmp);
							pq.add(gp);
						}	
					}
				}
			}
		}
		return null;
		
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
		
		// Hook for visualization.  
		initiate();
		if(start == null || goal == null) {
			System.out.println("Cannot find path");
			return null;
		}
		
		if(start == goal) {
			System.out.println("Start and end nodes are the same");
			return null;
		}
		
		MapNode startNode = graph.get(start);
		MapNode endNode = graph.get(goal);
		
		//for finding out the smallest
		PriorityQueue<MapNode> pq = new PriorityQueue<MapNode>(new AstarComparator());
		
		//To maintain visited nodes
		HashSet<GeographicPoint> vis = new HashSet<>();
		
		//To have parent array
		HashMap<MapNode, MapNode> par = new HashMap<>();
		
		//to check whether we have found solution or not.
		int count = 0;
		
		startNode.setWeight(0.0);
		startNode.setDistance(startNode);
		pq.offer(startNode);
		
		while(!pq.isEmpty()) {
			MapNode tmp=pq.poll();
			double cost=tmp.getWeight();
			
			if(!vis.contains(tmp.getLocation())) {
				vis.add(tmp.getLocation());
				nodeSearched.accept(tmp.getLocation());
				
				++count;
				//System.out.println(tmp.getLocation());
				if(endNode.equals(tmp)) {
					System.out.println("Astar :"+count);
					return path(startNode,endNode,par);
				}
				
				for(MapEdges e:tmp.getEdge()) {
					MapNode gp=e.getOppositeEnd(tmp);
					if(!vis.contains(gp.getLocation())) {
						double check=cost+e.getDistance();
						if(gp.getWeight()>check) {
							gp.setWeight(check);
							gp.setDistance(endNode);
							par.put(gp,tmp);
							pq.offer(gp);
						}
					}
				}
			}
		}
		return null;		
	}
	
	public List<GeographicPoint> searchByTime(GeographicPoint start, GeographicPoint goal) {
		Consumer<GeographicPoint> temp = (x) -> {};
        return searchByTime(start, goal, temp);
	}
	
	//method to search the best route in an even effective way by computing the time required / Dijkstra
	public List<GeographicPoint> searchByTime(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
		if(start==null || goal==null) {
			System.out.println("Path cannot be found out");
			return null;
		}
		if(start==goal) {
			System.out.println("Starting and ending are same");
			return null;
		}
		
		MapNode startNode = graph.get(start);
		MapNode endNode = graph.get(goal);
		
		if(startNode == null || endNode == null) {
			System.out.println("Path cannot be found out");
			return null;
		}
		
		//storing nodes
		PriorityQueue<MapNode> pq = new PriorityQueue<>(new DijkstraComparator());
		
		//storing visited locations
		HashSet<GeographicPoint> vis = new HashSet<>();
		
		HashMap<MapNode, MapNode> par=new HashMap<>();
		int count=0;
		double time=0;
		
		//time as weight
		initiate();
		startNode.setWeight(0.0);
		pq.offer(startNode);
		
		while(!pq.isEmpty()) {
			MapNode curr  = pq.remove();
			double cost = curr.getWeight();
			if(!vis.contains(curr.getLocation())) {
				vis.add(curr.getLocation());
				nodeSearched.accept(curr.getLocation());
				time+=curr.getWeight();
				++count;
				
				if(goal.equals(curr.getLocation())) {
					System.out.println("Time required : " + time + " " + endNode.getWeight());
					System.out.println("Nodes visited : " +count);
					return path(startNode,endNode,par);
				}
				
				for(MapEdges e : curr.getEdge()) {
					MapNode gp = e.getOppositeEnd(curr);
					if(!vis.contains(gp.getLocation())) {
						double check = cost + e.getTime();
						if(gp.getWeight() > check) {
							gp.setWeight(check);
							pq.offer(gp);
							par.put(gp, curr);
						}
					}
				}
					
			}
		}
		return null;
	}
		
	public List<GeographicPoint> AstarTime(GeographicPoint start, GeographicPoint goal) {
		Consumer<GeographicPoint> temp = (x) -> {};
       return AstarTime(start, goal, temp);
	}
	
	//method to search the best route in an even effective way by computing the time required
		public List<GeographicPoint> AstarTime(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
			if(start == null || goal == null) {
				System.out.println("Path cannot be found out");
				return null;
			}
			if(start == goal) {
				System.out.println("Starting and ending are same");
				return null;
			}
			
			MapNode startNode = graph.get(start);
			MapNode endNode = graph.get(goal);
			
			if(startNode == null || endNode == null) {
				System.out.println("Path cannot be found out");
				return null;
			}
			
			//storing nodes
			PriorityQueue<MapNode> pq = new PriorityQueue<>(new AstarTimeComparator());
			
			//storing visited locations
			HashSet<GeographicPoint> vis = new HashSet<>();
			
			HashMap<MapNode,MapNode> par = new HashMap<>();
			int count=0;
			double time=0;
			
			//time as weight
			initiate();
			startNode.setWeight(0.0);
			pq.offer(startNode);
			
			while(!pq.isEmpty()) {
				MapNode curr = pq.remove();
				double cost = curr.getWeight();
				if(!vis.contains(curr.getLocation())) {
					vis.add(curr.getLocation());
					nodeSearched.accept(curr.getLocation());
					time += curr.getWeight();
					++count;
					
					if(goal.equals(curr.getLocation())) {
						System.out.println("Time required : " + time + " " + endNode.getWeight());
						System.out.println("Nodes visited : " + count);
						return path(startNode,endNode,par);
					}
					
					for(MapEdges e : curr.getEdge()) {
						MapNode gp = e.getOppositeEnd(curr);
						if(!vis.contains(gp.getLocation())) {
							double check = cost + e.getTime();
							if(gp.getWeight() > check) {
								gp.setWeight(check);
								gp.setDistance(endNode);
								gp.setTimeWeight(gp.getDistWeight()/180.0);
								pq.offer(gp);
								par.put(gp, curr);
							}
						}
					}
						
				}
			}
			return null;
		}
		
		
	public static void main(String[] args) {
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
	    GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
//		MapGraph simpleTestMap = new MapGraph();
//		//GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
//		
//		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
//		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
//		
//		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
//		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
//		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
//		
//		
//		MapGraph testMap = new MapGraph();
//		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
//		
//		// A very simple test using real data
//		testStart = new GeographicPoint(32.869423, -117.220917);
//		testEnd = new GeographicPoint(32.869255, -117.216927);
//		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
//		testMap.dijkstra(testStart,testEnd);
//		testMap.aStarSearch(testStart,testEnd);
//				
//		//GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
//		// A slightly more complex test using real data
//		testStart = new GeographicPoint(32.8674388, -117.2190213);
//		testEnd = new GeographicPoint(32.8697828, -117.2244506);
//		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
//		testroute = testMap.dijkstra(testStart,testEnd);
//		testroute2 = testMap.aStarSearch(testStart,testEnd);
//		
//		
//		// Use this code in Week 3 End of Week Quiz 
//		MapGraph theMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
//		System.out.println("DONE.");
//
//		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
//		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
//		List<GeographicPoint> route = theMap.dijkstra(start,end);
//		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
	}
	
}
