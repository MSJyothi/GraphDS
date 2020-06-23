package com.jyothi.graphs;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.Stack;

/**
 * A Weighted Graph implementation to find the shortest path between two nodes,
 * cycle detection and finding the minimum spanning tree
 */
public class WeightedGraph {

	private Map<String, Node> nodes = new HashMap<>();

	private class Node {

		private String label;
		private List<Edge> edges = new ArrayList<>();

		public Node(String label) {
			this.label = label;
		}

		@Override
		public String toString() {
			return label;
		}

		public List<Edge> getEdges() {
			return edges;
		}

		public void addEdge(Node to, int weight) {
			edges.add(new Edge(this, to, weight));
		}

	}

	private class Edge {
		private Node from;
		private Node to;
		private int weight;

		public Edge(Node from, Node to, int weight) {
			this.from = from;
			this.to = to;
			this.weight = weight;
		}

		public String toString() {
			return from + "->" + to;
		}

	}

	public Map<String, Node> getNodes() {
		return nodes;
	}

	public void addNode(String label) {
		nodes.putIfAbsent(label, new Node(label));
	}

	public boolean containsNode(String label) {
		return nodes.containsKey(label);
	}

	public void addEdge(String from, String to, int weight) {

		Node fromNode = nodes.get(from);
		if (fromNode == null)
			throw new IllegalStateException();

		Node toNode = nodes.get(to);
		if (toNode == null)
			throw new IllegalStateException();

		fromNode.addEdge(toNode, weight);
		toNode.addEdge(fromNode, weight);

	}

	public void print() {
		for (var node : nodes.values()) {
			var edges = node.getEdges();
			if (!edges.isEmpty())
				System.out.println(node + " is connected to " + edges);

		}
	}

	private class NodeEntry {
		private Node node;
		private int priority;

		public NodeEntry(Node node, int priority) {
			this.node = node;
			this.priority = priority;
		}

		public String toString() {
			return node + "-" + priority;
		}

	}

	/**
	 * Provides the shortest path between the two nodes using Dijkstra's shortest path algorithm
	 */
	public Path getShortestPath(String from, String to) {

		Node fromNode = nodes.get(from);
		if (fromNode == null)
			throw new IllegalStateException();

		Node toNode = nodes.get(to);
		if (toNode == null)
			throw new IllegalStateException();

		Map<Node, Integer> distances = new HashMap<>();
		for (Node node : nodes.values()) {
			distances.put(node, Integer.MAX_VALUE);
		}

		distances.replace(fromNode, 0);

		Map<Node, Node> previousNodes = new HashMap<>();

		Set<Node> visitedNodes = new HashSet<>();

		PriorityQueue<NodeEntry> queue = new PriorityQueue<>(Comparator.comparingInt(ne -> ne.priority));
		queue.add(new NodeEntry(fromNode, 0));

		while (!queue.isEmpty()) {
			var currentNode = queue.remove().node;

			visitedNodes.add(currentNode);
			for (var edges : currentNode.getEdges()) {
				if (visitedNodes.contains(edges.to))
					continue;

				int newDistance = edges.weight + distances.get(currentNode);
				if (newDistance < distances.get(edges.to)) {
					distances.replace(edges.to, newDistance);
					queue.add(new NodeEntry(edges.to, newDistance));
					previousNodes.put(edges.to, currentNode);
				}
			}

		}

		return buildPath(toNode, previousNodes);

	}

	private Path buildPath(Node toNode, Map<Node, Node> previousNodes) {
		Stack<Node> stack = new Stack<Node>();

		stack.push(toNode);
		var previous = previousNodes.get(toNode);
		while (previous != null) {
			stack.push(previous);
			previous = previousNodes.get(previous);
		}

		Path path = new Path();
		while (!stack.isEmpty()) {
			path.add(stack.pop().label);
		}

		return path;
	}

	/**
	 * Finds whether a graph contains a cycle
	 */
	public boolean hasCycle() {

		Set<Node> visited = new HashSet<Node>();

		for (var node : nodes.values()) {
			if (!visited.contains(node) && hasCycle(node, null, visited))
				return true;
		}

		return false;

	}

	private boolean hasCycle(Node node, Node parent, Set<Node> visited) {

		visited.add(node);

		for (var edge : node.getEdges()) {

			if (parent == edge.to) {
				continue;
			}

			if (visited.contains(edge.to)) {
				return true;
			}

			if (hasCycle(edge.to, node, visited)) {
				return true;
			}

		}

		return false;

	}

	/**
	 * Finds the minimum spanning tree of a graph using Prim's algorithm.
	 */
	public WeightedGraph getMinimumSpanningTree() {

		WeightedGraph tree = new WeightedGraph();
		if (nodes.isEmpty())
			return tree;

		PriorityQueue<Edge> queue = new PriorityQueue<>(Comparator.comparingInt(ne -> ne.weight));

		Node startNode = nodes.values().iterator().next();

		// The node in the minimum spanning tree should be a new node
		tree.addNode(startNode.label);
		queue.addAll(startNode.edges);

		if (queue.isEmpty()) {
			return tree;
		}

		while (tree.nodes.size() < nodes.size()) {

			// for any disconnected nodes
			if (queue.isEmpty()) {
				return tree;
			}

			Edge minEdge = queue.remove();
			Node nextNode = minEdge.to;

			if (tree.containsNode(nextNode.label)) {
				continue;
			}

			tree.addNode(minEdge.to.label);
			tree.addEdge(minEdge.from.label, nextNode.label, minEdge.weight);

			for (var edge : nextNode.getEdges()) {
				if (!(tree.containsNode(edge.to.label)))
					queue.add(edge);
			}

		}

		return tree;

	}

}
