package reqextension;
import geography.GeographicPoint;
import java.util.List;
import java.util.ArrayList;
import java.util.Comparator;

public class MapNode {
	
	private GeographicPoint location;
	private List<MapEdges> edges;
	private double weight;
	private double distWeight;
	private double timeWeight;
	
	public MapNode(GeographicPoint location) {		
		edges = new ArrayList<>();
		this.location = location;	
		distWeight = (double)Integer.MAX_VALUE;
	}	
	
	public GeographicPoint getLocation() {
		return this.location;
	}
		
	public void addEdge(MapEdges e) {
		this.edges.add(e);
	}
	
	public void setTimeWeight(double timeValue) {
		this.timeWeight = timeValue;
	}
		
	public double getTimeWeight() {
		return this.timeWeight;
	}
	
	public List<MapEdges> getEdge() {
		return this.edges;
	}
	
	public double getWeight() {
		return this.weight;
	}
	
	public double getDistWeight() {
		return distWeight;
	}
	
	public void setDistance(MapNode goal) {
		distWeight = this.location.distance(goal.getLocation());
	}
	
	public void setValue() {
		distWeight = (double)Integer.MAX_VALUE;
	}
	
	public void setWeight(double theWeight) {
		this.weight = theWeight;
	}
}