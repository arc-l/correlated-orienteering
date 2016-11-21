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

import java.io.Serializable;
import java.util.HashSet;
import java.util.Set;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.Vector;

/**
 * 
 * @author Jingjin
 *
 * A basic implementation of a graph. Note that the vertices must have 
 * consecutive ids starting from 0.  
 * 
 */
public class Graph implements Serializable{
	private static final long serialVersionUID = 1L;
	
	// A local "infinity" 
	public static final double INFINITY = 10000000000.;
	
	// Id-vertex map
	public SortedMap<Integer, Vertex> vertexMap = new TreeMap<Integer, Vertex>();

	// Edge lengths between all pairs of vertices
	public double[][] edgeLengths = null; 
	
	// Vertex order
	private int[] vertexOrder = null;
	
	/**
	 * Add a vertex to a graph
	 * @param v
	 */
	public void addVertex(Vertex v){
		vertexMap.put(v.id, v);
	}
	
	/**
	 * Retrieve a specific vertex
	 * @param id
	 * @return
	 */
	public Vertex getVertex(int id){
		return vertexMap.get(id);
	}
	
	/**
	 * Retrieve all vertices
	 * @return
	 */
	public Vertex[] getVertices(){
		Vertex[] vs = vertexMap.values().toArray(new Vertex[0]);
		if(vertexOrder == null){
			return vs;
		}
		else{
			Vertex[] vsRet = new Vertex[vs.length];
			for(int i = 0; i < vs.length; i++){
				vsRet[i] = vs[vertexOrder[i]];
			}
			return vsRet;
		}
	}
	
	/**
	 * Retrieve all vertices in a 2D grid
	 * @param n
	 * @return
	 */
	public Vertex[][] get2DVertices(int n){
		Vertex[] vertices = getVertices();
		Vertex[][] ret = new Vertex[n][n];
		for(int row = 0;row < n; row ++){
			for(int col = 0; col < n; col ++){
				ret[row][col] = vertices[row*n + col];
			}
		}
		return ret;
	}

	/**
	 * Update the order of vertices. First vertex is used as the base vertex
	 * @param order
	 */
	public void setVertexOrder(int order[]){
		vertexOrder = order;
	}

	/**
	 * Check edge length constraint - whether the length connecting the two 
	 * vertices is smaller than the given length
	 * @param id1
	 * @param id2
	 * @param length
	 * @return
	 */
	public boolean hasEdge(int id1, int id2, double length){
		if(this.edgeLengths[id1][id2] <= length){
			return true;
		}
		else{
			return false;
		}
	}
	
	/**
	 * Check edge length constraint - whether there is an edge between the two
	 * vertices
	 * @param id1
	 * @param id2
	 * @return
	 */
	public boolean hasEdge(int id1, int id2){
		if(Double.isInfinite(this.edgeLengths[id1][id2])){
			return false;
		}
		else{
			return true;
		}
	}

	/**
	 * Randomize vertex locations 
	 */
	public void randomizeVertexLocations(double variance){
		Vertex[] vertices = this.getVertices();
		for(Vertex v:vertices){
			v.x += (Math.random() - 0.5)*variance;
			v.y += (Math.random() - 0.5)*variance;
		}

		// Update vertex-vertex distance
		computeEdgeLengths();
	}
	
	private Vector<Vertex> getDiagonalNeighbors(Vertex v, int nRow, int nCol){
		Vector<Vertex> nbrs = new Vector<Vertex>();
		
		// Collect diagonal neighbor ids
		Vector<Integer> dnIdVec = new Vector<Integer>(); 
		if(v.row > 0 && v.col > 0)
			dnIdVec.add((v.row - 1)*nCol + v.col - 1);
		if(v.row > 0 && v.col < nCol - 1)
			dnIdVec.add((v.row - 1)*nCol + v.col + 1);
		if(v.row > nRow - 1 && v.col > 0)
			dnIdVec.add((v.row + 1)*nCol + v.col - 1);
		if(v.row > nRow - 1 && v.col < nCol - 1)
			dnIdVec.add((v.row + 1)*nCol + v.col + 1);
		
		// Add only valid ones to the return vector
		for(int i = 0; i < dnIdVec.size(); i ++){
			int id = dnIdVec.get(i);
			if(id >= 0 && id < this.vertexMap.size()){
				nbrs.add(this.getVertex(id));
			}
		}
		return nbrs;
	}
	
	/**
	 * Randomly add and remove some edges, limited to 8 neighbors 
	 */
	public void randomizeEdges(double deleteProbability,
			double addProbability){
		// Retrieve all vertices 
		Vertex[] vertices = this.getVertices();
		
		// Randomize the order of these vertices; also collect max row & col
		Vector<Vertex> vVec = new Vector<Vertex>();
		int nRow = 1, nCol = 1;
		for(Vertex v: vertices){
			vVec.insertElementAt(v, (int)(Math.random()*(vVec.size()+1)));
			if(v.row+1 > nRow) nRow = v.row + 1;
			if(v.col+1 > nCol) nCol = v.col + 1;
		}
		vertices = vVec.toArray(new Vertex[0]);
		
		// For each vertex, first delete, with some probability, its current
		// edges
		for(Vertex v:vertices){
			Vector<Integer> newIdVec = new Vector<Integer>();
			Vector<Vertex> newVertexVec = new Vector<Vertex>();
			// Iterate through neighbors
			for(int i = 0; i < v.neighbors.length; i++){
				if(Math.random() >= deleteProbability/2){
					// Keeping the vertex, the probability of 
					// deleteProbability/2 is used because each edge can be 
					// checked twice
					newIdVec.add(v.id);
					newVertexVec.add(v.neighbors[i]);
				}
				else{
					// Removing the vertex, which means that we need to 
					// handle the neighbor
					v.neighbors[i].removeNeighbor(v.id);
				}
			}
			
			// Update neighbors
			v.neighbors = newVertexVec.toArray(new Vertex[0]);
			v.nbrsIds = new int[v.neighbors.length];
			for(int i = 0; i < v.nbrsIds.length; i ++){
				v.nbrsIds[i] = v.neighbors[i].id;
			}
		}
		// Update vertex-vertex distance
		computeEdgeLengths();

		// For each vertex, add with some probability some edges to its diagonal
		// neighbors. 
		for(Vertex v:vertices){
			// Collect possible diagonal vertices
			Vector<Vertex> diagVVec = getDiagonalNeighbors(v, nRow, nCol);
			for(int i = 0; i < diagVVec.size();i ++){
				if(Math.random() < addProbability/2){
					Vertex nbrV = diagVVec.get(i);
					// Make sure we don't have the other diagonal edge already
					if(nbrV.row > v.row){
						int vd1id = (nbrV.row  - 1)*nCol + nbrV.col;
						int vd2id = (v.row + 1)*nCol + v.col;
						if(vertexMap.get(vd1id).hasNeighbor(vd2id))continue;
					}
					else{
						int vd1id = (nbrV.row  + 1)*nCol + nbrV.col;
						int vd2id = (v.row - 1)*nCol + v.col;
						if(vertexMap.get(vd1id).hasNeighbor(vd2id))continue;
					}
					
					
					nbrV.addNeighbor(v.id, v);
					v.addNeighbor(nbrV.id, nbrV);
					edgeLengths[v.id][nbrV.id] = edgeLengths[nbrV.id][v.id] =
							Math.sqrt((v.x-nbrV.x)*(v.x-nbrV.x) +
									(v.y - nbrV.y)*(v.y-nbrV.y));
					
				}
			}
		}

		for(Vertex v:vertices){
			v.updateWeights();
		}
		
		// Update vertex-vertex distance
		computeEdgeLengths();
	}
	
	/**
	 * Randomize importance of vertices 
	 */
	public void randomizeVertexImportance(double variance){
		Vertex[] vertices = this.getVertices();
		for(Vertex v:vertices){
			v.importance = 1 + (Math.random() - 0.5)*variance;
		}
	}
	
	/**
	 * Print nodes
	 */
	public void print(){
		Vertex[] vertices = this.getVertices();
		for(Vertex v:vertices){
			System.out.printf("v[%3d]:%7.4f, %7.4f  Nbrs:", v.id, v.x, v.y);
			for(int i = 0; i < v.neighbors.length; i ++){
				System.out.print("[" + v.neighbors[i].id +
					(v.beta != null? "," + v.beta[i+1]:"") + "]");
			}
			System.out.println();
		}
	}
	
	/**
	 * Compute Euclidean edge lengths based on vertex x & y
	 */
	public void computeEdgeLengths(){
		edgeLengths = new double[vertexMap.size()][vertexMap.size()];
		for(int i = 0; i < edgeLengths.length; i ++){
			for(int j = 0; j < edgeLengths[i].length; j ++){
				if(i == j){
					edgeLengths[i][j] = Graph.INFINITY;
				}
				else{
					Vertex vi = vertexMap.get(i);
					Vertex vj = vertexMap.get(j);
					edgeLengths[i][j] = Math.sqrt((vi.x - vj.x)*(vi.x - vj.x) 
							+ (vi.y - vj.y)*(vi.y - vj.y));
				}
			}
		}
	}
	
	/**
	 * Update vertex neighbor array based on neighbor id array 
	 */
	public void updateNeighbors(){
		Vertex vertices[] = getVertices();
		for(int i = 0; i < vertices.length; i ++){
			Vertex v = vertices[i];
			v.neighbors = new Vertex[v.nbrsIds.length];
			for(int n = 0; n < v.neighbors.length; n ++){
				v.neighbors[n] = getVertex(v.nbrsIds[n]);
			}
		}
	}

//******************************************************************************
//  Static method for creating graph instances
//******************************************************************************
	
	/**
	 * Create a grid graph of n x n
	 * @param n
	 * @return
	 */
	public static Graph getnxnGrid(int n){
		return getnxmGrid(n, n);
	}
	
	/**
	 * Create a grid graph of n x n
	 * @param n
	 * @return
	 */
	public static Graph getnxmGrid(int n, int m){
		Graph g = new Graph();
		
		// Create n*n vertices
		Vertex[] vertices = new Vertex[n*m];
		for(int row = 0; row < n; row ++){
			for(int col = 0; col < m; col ++){
				vertices[row*m + col] = new  Vertex(row*m + col);
				vertices[row*m + col].row = row;
				vertices[row*m + col].col = col;
			}
		}

		// Populate vertex structures
		for(int row = 0; row < n; row ++){
			for(int col = 0; col < m; col ++){
				// Adding connected vertices
				Set<Vertex> nbrSet = new HashSet<Vertex>();
				// Above
				if(row > 0) nbrSet.add(vertices[(row-1)*m + col]);
				// Below 
				if(row < n - 1) nbrSet.add(vertices[(row+1)*m + col]);
				// Left
				if(col > 0) nbrSet.add(vertices[row*m + col - 1]);
				// Right 
				if(col < n - 1) nbrSet.add(vertices[row*m + col + 1]);
				vertices[row*m + col].neighbors = nbrSet.toArray(new Vertex[0]);
				vertices[row*m + col].nbrsIds = new int[nbrSet.size()];
				for(int nbr = 0; nbr < vertices[row*m + col].nbrsIds.length;
						nbr++){
					vertices[row*m + col].nbrsIds[nbr] =
							vertices[row*m + col].neighbors[nbr].id;
				}
				
				// Set weights to be 1/number of neighbors
				vertices[row*m + col].weights = 
						new double[vertices[row*m + col].neighbors.length];
				for(int i = 0; i < vertices[row*m + col].weights.length; i ++){
					vertices[row*m + col].weights[i] = 
							1.0/vertices[row*m + col].weights.length;
				}
				
				// Set coordinates
				vertices[row*m + col].x = col;
				vertices[row*m + col].y = row;
			}
		}

		// Populate vertices to the graph
		for(Vertex vertex:vertices){
			g.addVertex(vertex);
		}
		
		// Compute edge lengths
		g.computeEdgeLengths();
		return g;
	}
	
	/**
	 * Create a 3 x 3 grid graph with a specific set of parameters
	 * @return
	 */
	public static Graph getGridTestCaseGraph(){
		
		Graph g = Graph.getnxnGrid(3);
		Vertex[] vertices = g.getVertices();
		vertices[0].x = -4.022978910685923;
		vertices[0].y = -4.402259622850392;
		vertices[0].neighbors = 
				new Vertex[]{vertices[1],vertices[3]};
		vertices[0].beta = new double[]{0,
				0.9424144774496166,-0.0409631449936203};
		vertices[1].x = -2.818555252451967;
		vertices[1].y = -3.524192086578621;
		vertices[1].neighbors = 
				new Vertex[]{vertices[0],vertices[4],vertices[2]};
		vertices[1].beta = new double[]{0,
				0.133464378899585,0.8295374197795726,0.086513196571021};
		vertices[2].x = 1.0927646720269781;
		vertices[2].y = -3.9422579331898637;
		vertices[2].neighbors = 
				new Vertex[]{vertices[1],vertices[5]};
		vertices[2].beta = new double[]{0,
				3.8473660453572593,-3.923633318469499};
		vertices[3].x = -4.702814891487349;
		vertices[3].y = -1.0858292085252426;
		vertices[3].neighbors = 
				new Vertex[]{vertices[0],vertices[6],vertices[4]};
		vertices[3].beta = new double[]{0,
				0.4602541250893958,0.08038744870310445,0.08825078456823454};
		vertices[4].x = -2.2730333673395577;
		vertices[4].y = -2.2284201579185043;
		vertices[4].neighbors = 
				new Vertex[]{vertices[7],vertices[1],vertices[3],vertices[5]};
		vertices[4].beta = new double[]{0,
				-0.024717126029422002,0.42860753731422596,
				0.48695186758645836,0.3851007472962927};
		vertices[5].x = 0.20637341860850333;
		vertices[5].y = -1.323427262931723;
		vertices[5].neighbors = 
				new Vertex[]{vertices[8],vertices[4],vertices[2]};
		vertices[5].beta = new double[]{0,
				0.13932477220162953,0.6253403649925722,0.158416798995787};
		vertices[6].x = -4.236887804296595;
		vertices[6].y = 1.6334723527277848;
		vertices[6].neighbors = 
				new Vertex[]{vertices[7],vertices[3]};
		vertices[6].beta = new double[]{0,
				0.669758432821785,-0.11188780170656007};
		vertices[7].x = -1.6673269312616246;
		vertices[7].y = 1.7614701086302063;
		vertices[7].neighbors = 
				new Vertex[]{vertices[8],vertices[6],vertices[4]};
		vertices[7].beta = new double[]{0,
				0.4898133499112793,1.0547355481809366,-0.18335781634399542};
		vertices[8].x = 0.9945540410914664;
		vertices[8].y = 0.36168318681546796;
		vertices[8].neighbors = 
				new Vertex[]{vertices[7],vertices[5]};
		vertices[8].beta = new double[]{0,
				0.5833029054129802,0.6145681406675388};
		g.computeEdgeLengths();
		
		//Update weights
		Vertex[][] vs = g.get2DVertices(3);
		// We got all data, now do regression to populate the weights
		for(int row = 0;row < 3; row ++){
			for(int col = 0; col < 3; col ++){
				// Get all neighbors
				Vertex nbrs[] = vs[row][col].neighbors;
				
				// Build weights. First sum up all coeffs
				double sumCoeff = 0;
				for(int i = 0; i < nbrs.length; i ++){
					sumCoeff += Math.abs(vs[row][col].beta[i + 1]); 
				}
				for(int i = 0; i < nbrs.length; i ++){
					vs[row][col].weights[i] = 
							Math.abs(vs[row][col].beta[i + 1])/sumCoeff;
				}
			}
		}
		return g;
	}

	/**
	 * Get the test case graph for the paper 
	 * @return
	 */
	public static Graph getIrregularTestCaseGraph(){

		Vertex vs[]= new Vertex[18];
		vs[0] = new Vertex(0,new int[]{1,11,17}, new double[]{},1.56,7.3);
		vs[1] = new Vertex(1,new int[]{0,2,11}, new double[]{},1.26,2.7);
		vs[2] = new Vertex(2,new int[]{1,3,9,10}, new double[]{},4.64,1.04);
		vs[3] = new Vertex(3,new int[]{2,4,8}, new double[]{},8.34,1.58);
		vs[4] = new Vertex(4,new int[]{3,5,7}, new double[]{},11.7,1.14);
		vs[5] = new Vertex(5,new int[]{4,6}, new double[]{},14.18,2.56);
		vs[6] = new Vertex(6,new int[]{5,7}, new double[]{},13.1,4.94);
		vs[7] = new Vertex(7,new int[]{4,6,8,13,14}, new double[]{},10.76,3.92);
		vs[8] = new Vertex(8,new int[]{3,7,9}, new double[]{},8.12,3.44);
		vs[9] = new Vertex(9,new int[]{2,8,12,13}, new double[]{},6.2,3.72);
		vs[10] = new Vertex(10,new int[]{2,11}, new double[]{},3.86,3.06);
		vs[11] = new Vertex(11,new int[]{0,1,10,12}, new double[]{},2.82,4.68);
		vs[12] = new Vertex(12,new int[]{9,11,17}, new double[]{},4.58,5.78);
		vs[13] = new Vertex(13,new int[]{7,9,14,16}, new double[]{},7.96,6.12);
		vs[14] = new Vertex(14,new int[]{7,13,15}, new double[]{},10.1,6.5);
		vs[15] = new Vertex(15,new int[]{14}, new double[]{},12.88,8.52);
		vs[16] = new Vertex(16,new int[]{13,17}, new double[]{},6.76,8.82);
		vs[17] = new Vertex(17,new int[]{0,12,16}, new double[]{},4.18,8.08);
		for(int i = 0; i <vs.length; i ++){
			vs[i].updateNeighbors(vs);
		}
		for(int i = 0; i <vs.length; i ++){
			vs[i].updateWeights();
		}
	
		Graph g = new Graph();
		for(int i = 0; i <vs.length; i++){
			g.addVertex(vs[i]);
		}
		g.computeEdgeLengths();
		
		return g;
	}
	
//******************************************************************************
//  Test main
//******************************************************************************
	
	public static void main(String argv[]){
		Graph g = Graph.getnxnGrid(5);
		g.randomizeVertexLocations(0.2);
		g.print();
	}
	
	
	
}
