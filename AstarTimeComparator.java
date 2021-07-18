package reqextension;

import java.util.Comparator;

class AstarTimeComparator implements Comparator<MapNode> {
	public int compare(MapNode a,MapNode b) {
		if(a.getWeight() + a.getTimeWeight() < b.getWeight() + b.getTimeWeight())
			return -1;
		else if(a.getWeight() + a.getTimeWeight() > b.getWeight() + b.getTimeWeight())
			return 1;
		else
			return 0;
	}
}
