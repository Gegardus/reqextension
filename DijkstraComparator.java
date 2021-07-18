package reqextension;

import java.util.Comparator;

 class DijkstraComparator implements Comparator<MapNode> {
	public int compare(MapNode a,MapNode b) {
		if(a.getWeight() < b.getWeight())
			return -1;
		else if(a.getWeight() > b.getWeight())
			return 1;
		else
			return 0;
		
	}
}
