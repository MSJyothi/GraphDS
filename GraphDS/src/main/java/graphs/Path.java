package main.java.graphs;

import java.util.ArrayList;
import java.util.List;

/* a util class */
public class Path {
	List<String> nodes= new ArrayList<>();
	
	
	public void add(String label) {
		nodes.add(label);
	}
	
	@Override
	public String toString() {
		return nodes.toString();
	}

}
