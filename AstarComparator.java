package reqextension;

import java.util.Comparator;

class AstarComparator implements Comparator<MapNode> {
	public int compare(MapNode a, MapNode b) {
		if(a.getWeight() + a.getDistWeight() < b.getWeight() + b.getDistWeight())
			return -1;
		else if(a.getWeight() + a.getDistWeight() > b.getWeight() + b.getDistWeight())
			return 1;
		else
			return 0;
	}
}
