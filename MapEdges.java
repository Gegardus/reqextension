package reqextension;
import geography.GeographicPoint;

public class MapEdges {
	
	private MapNode start; 
	private MapNode end;
	private String streetName; 
	private String streetType;
	private double distance;
	private double speed;
	
	public MapEdges(MapNode start, MapNode end, String streetName, String streetType, double distance) {
		this.start = start;
		this.end = end;
		this.streetType = streetType;
		this.streetName = streetName;
		this.distance = distance;
		setSpeed();
	}
	
	public void setSpeed() {
		switch(streetType) {
			case "motorway":
				speed = 120.0;
				break;			
			case "city street":
				speed = 80.0;
				break;
			case "residential":
				speed = 50.0;
				break;
			case "connector":
				speed = 60.0;
				break;			
			case "living_street":
				speed = 30.0;
				break;
			default:
				speed = 50.0;
		}
	}
	
	public double getSpeed() {
		return speed;
	}
	
	public String getStreetType() {
		return streetType;
	}
	
	public MapNode getOppositeEnd(MapNode mp) {
		if(mp.equals(start))
			return end;
		else if(end.equals(mp))
			return start;
		else 
			throw new IllegalArgumentException();
	}
	
	public double getDistance() {
		return this.distance;
	}
	
	public double getTime() {
		return distance/speed;
	}
	
	public void setDistance(double theDistance) {
		this.distance = theDistance;
	}

}