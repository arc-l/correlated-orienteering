/***********************************************************************
 *  All components of this library are licensed under the BSD 3-Clause 
 *  License.
 *
 *  Copyright (c) 2015-, Algorithmic Robotics and Control Group @Rutgers 
 *  (https://arc.cs.rutgers.edu). All rights reserved.
 *  
 *	Redistribution and use in source and binary forms, with or without
 *	modification, are permitted provided that the following conditions are
 *	met:
 *	
 *	Redistributions of source code must retain the above copyright notice,
 *	this list of conditions and the following disclaimer.  Redistributions
 *	in binary form must reproduce the above copyright notice, this list of
 *	conditions and the following disclaimer in the documentation and/or
 *	other materials provided with the distribution. Neither the name of
 *	Rutgers University nor the names of the contributors may be used to 
 *  endorse or promote products derived from this software without specific
 *  prior written permission.
 *	
 *	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *	A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *	HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *	SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *	LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


package edu.rutgers.cs.arc.qcop;

import java.util.HashSet;
import java.util.Vector;

import edu.rutgers.cs.arc.qcop.Graph;

/**
 * 
 * @author Jingjin
 *
 * Greedy algorithm for solving QCOP
 * 
 */
public class GreedyAlgorithm {

	/**
	 * Return the information gain of the current set of vertices
	 * 
	 * @param g
	 * @param visitedVertexSet
	 * @return
	 */
	public static double getCurrentInformationGain(Graph g,
			HashSet<Integer> visitedVertexSet) {
		double gain = 0;
		Vertex vertices[] = g.getVertices();
		for (int i = 0; i < vertices.length; i++) {
			int vid = vertices[i].id;
			// Check whether the vertex is in visited set
			if (visitedVertexSet.contains(vid)) {
				gain += 1;
			} else {
				// This vertex may have neighbors in the visited set. Simply
				// check all its neighbors
				Vertex v = g.getVertex(vid);
				for (int n = 0; n < v.neighbors.length; n++) {
					if (visitedVertexSet.contains(v.neighbors[n].id)) {
						gain += g.getVertex(v.neighbors[n].id).getWeight(v.id);
					}
				}
			}
		}
		return gain;
	}

	public static Vector<Vertex> solve(Graph g, int startVertex, double budget) 
	{

		Vector<Vertex> solutionVec = new Vector<Vertex>();

		// Maintain a visited set and an unvisited set
		HashSet<Integer> visitedVertexSet = new HashSet<Integer>();
		HashSet<Integer> unvisitedVertexSet = new HashSet<Integer>();
		visitedVertexSet.add(startVertex);
		Vertex vertices[] = g.getVertices();
		for (int i = 0; i < vertices.length; i++) {
			if (vertices[i].id != startVertex) {
				unvisitedVertexSet.add(vertices[i].id);
			}
		}

		Vector<Integer> pathVector = new Vector<Integer>();
		pathVector.add(startVertex);

		// Compute over all vertices the gain of information versus the 
		// distance cost 
		
		// First compute current information gain
		double pathLength = 0;
		while (true && pathVector.size() < vertices.length) {
			// Go through all unvisited vertices
			Integer unvisited[] = unvisitedVertexSet.toArray(new Integer[0]);
			double bestRatio = 0;
			int bestCandidate = -1;
			for (int i = 0; i < unvisited.length; i++) {
				// Compute the gain and cost. First the cost since we cannot go
				// over. The distance cost is the distance from the last vertex 
				// of the current path to the unvisited vertex and then from 
				// the unvisited vertex to the first vertex of the path
				int lastVertexId = pathVector.lastElement();
				double length = g.edgeLengths[lastVertexId][unvisited[i]]
						+ g.edgeLengths[unvisited[i]][pathVector.firstElement()];
				double lengthScaled = g.edgeLengths[lastVertexId][unvisited[i]];
				// + g.edgeLengths[unvisited[i]][pathVector.firstElement()]/
				// (pathVector.size()+1);
				
				if (length + pathLength > budget) {
					continue;
				}

				// OK, budget is not violated, we can compute the gain
				visitedVertexSet.add(unvisited[i]);
				double tempTotalGain = getCurrentInformationGain(g,
						visitedVertexSet);
				if (tempTotalGain / (lengthScaled + pathLength) > bestRatio) {
					bestRatio = tempTotalGain / (lengthScaled + pathLength);
					bestCandidate = unvisited[i];
				}
				visitedVertexSet.remove(unvisited[i]);
			}

			if (bestCandidate != -1) {
				pathLength += 
						g.edgeLengths[pathVector.lastElement()][bestCandidate];
				pathVector.add(bestCandidate);
				visitedVertexSet.add(bestCandidate);
				unvisitedVertexSet.remove(bestCandidate);
			} else {
				break;
			}
		}

		HashSet<Integer> visitedSet = new HashSet<Integer>();
		System.out.println("Total number of visited vertices: "
				+ pathVector.size());
		for (int i = 0; i < pathVector.size(); i++) {
			System.out.print(pathVector.get(i) + " ");
			solutionVec.add(g.getVertex(pathVector.get(i)));
			visitedSet.add(pathVector.get(i));
			if (i == pathVector.size() - 1) {
				solutionVec.add(g.getVertex(pathVector.get(0)));
			}
		}
		System.out.println();
		System.out.println("Total utility: " + computeUtility(g, visitedSet));

		return solutionVec;

	}

	public static double computeUtility(Graph g, HashSet<Integer> visitedIds) {
		Vertex vertices[] = g.getVertices();
		double utility = 0;
		for (Vertex v : vertices) {
			// Check whether the vertex is visited or not
			if (visitedIds.contains(v.id)) {
				// If visited, just add one to the total utility, representing
				// full information
				utility += 1;
			} else {
				// Not visited, check whether it has neighbors that is visited
				// and tally the scores
				for (int n = 0; n < v.neighbors.length; n++) {
					if (visitedIds.contains(v.neighbors[n].id)) {
						utility += v.neighbors[n].getWeight(v.id);
					}
				}
			}
		}
		return utility;
	}

	public static void main(String argv[]) {
		System.out.println("Testing single robot on randomized grids...");
		int[] gridSizes = new int[] { 3, 4, 5, 6, 7 };
		int startVertexId = 1;
		for (int s = 0; s < gridSizes.length; s++) {
			Graph g = Graph.getnxnGrid(gridSizes[s]);
			GreedyAlgorithm.solve(g, startVertexId, 10);
			// g.randomize();

		}
	}

}
