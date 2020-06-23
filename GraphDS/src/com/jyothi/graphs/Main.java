package com.jyothi.graphs;

public class Main {
	
	public static void main(String[] args) {
		
		   //Find the shortest path between two nodes
			WeightedGraph graph = new WeightedGraph();
			graph.addNode("A");
			graph.addNode("B");
			graph.addNode("C");
			graph.addNode("D");
			graph.addEdge("A", "B", 3);
			graph.addEdge("A", "C", 4);
			graph.addEdge("A", "D", 6);
			graph.addEdge("B", "D", 1);
			graph.print();
			System.out.println(graph.getShortestPath("A", "D"));

            //Find if a given graph contains cycle
			WeightedGraph graphWithCycle = new WeightedGraph();
			graphWithCycle.addNode("A");
			graphWithCycle.addNode("B");
			graphWithCycle.addNode("C");
			graphWithCycle.addEdge("A", "B", 1);
			graphWithCycle.addEdge("B", "C", 2);
			graphWithCycle.addEdge("A", "C", 1);
			graphWithCycle.print();
			System.out.println("Contains cycles : " + graphWithCycle.hasCycle());

			// Find minimum spanning tree
			WeightedGraph tree = new WeightedGraph();
			tree.addNode("A");
			tree.addNode("B");
			tree.addNode("C");
			tree.addNode("D");
			tree.addEdge("A", "B", 3);
			tree.addEdge("A", "C", 1);
			tree.addEdge("B", "C", 2);
			tree.addEdge("B", "D", 4);
			tree.addEdge("C", "D", 5);
			WeightedGraph minSpanningTree = tree.getMinimumSpanningTree();
			minSpanningTree.print();

		}

	

}
