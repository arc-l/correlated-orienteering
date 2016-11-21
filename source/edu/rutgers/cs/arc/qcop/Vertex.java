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

import java.util.Vector;

/**
 * 
 * @author Jingjin
 *
 * An implementation of a basic graph vertex containing node importance and 
 * neighbor influences. Generally node ids should starts from 0 and consecutive.
 * 
 */
public class Vertex {
	
	// Node id
	public int id;
	
	// Neighbors and neighbor vertex ids
	public Vertex[] neighbors;
	public int[] nbrsIds;
	
	// Weights. This encodes the information a neighbor has over the current 
	// node
	public double[] weights;
	
	// Row and column of vertex for regular grids
	public int row = -1, col = -1;
	
	// Node physical location in 2D
	public double x, y;
	
	// Node relative importance
	public double importance = 1;
	
	// Beta 
	public double beta[];
	
	// Constructors
	public Vertex(int id){this.id = id;}
	public Vertex(int id, int[] neighbors, double x, double y) {
		this(id, neighbors, null, x, y);
	}
	
	public Vertex(int id, int[] neighbors, double[] weights, 
			double x, double y){
		super();
		this.id = id;
		this.nbrsIds = neighbors;
		this.weights = weights;
		if(neighbors != null && weights == null){
			this.weights = new double[neighbors.length];
		}
		this.x = x;
		this.y = y;
	}
	
	/**
	 * Update neighbors based on an array of Vertex
	 * @param vs
	 */
	public void updateNeighbors(Vertex[] vs){
		Vector<Vertex> nbrVec = new Vector<Vertex>();
		for(int i = 0; i < nbrsIds.length; i ++){
			nbrVec.add(vs[nbrsIds[i]]);
		}
		neighbors = nbrVec.toArray(new Vertex[0]);
	}
	
	/**
	 * Check whether the current vertex has a neighbor with the given id 
	 * 
	 * @param id
	 * @return
	 */
	public boolean hasNeighbor(int id){
		for(int i = 0; i < this.nbrsIds.length; i ++){
			if(id == nbrsIds[i]){
				return true;
			}
		}
		return false;
	}
	
	/**
	 * Add a neighbor vertex
	 * @param id
	 */
	public void addNeighbor(int id, Vertex v){
		Vector<Vertex> nbrVec = new Vector<Vertex>();
		Vector<Integer> nbrIdVec = new Vector<Integer>();

		// Preserve a neighbor if it is not to be removed
		for(int i = 0; i < neighbors.length; i ++){
			nbrVec.addElement(this.neighbors[i]);
		}
		nbrIdVec.add(id);
		nbrVec.add(v);
		
		// Update neighbors
		this.neighbors = nbrVec.toArray(new Vertex[0]);
		
		int[] newNbrs = new int[this.nbrsIds.length + 1];
		for(int i =0; i < newNbrs.length -1; i++){
			newNbrs[i] = this.nbrsIds[i];
		}
		newNbrs[this.nbrsIds.length] = id;
		this.nbrsIds = newNbrs;
	}
	

	/**
	 * Remove a neighbor vertex
	 * @param id
	 */
	public void removeNeighbor(int id){
		Vector<Vertex> nbrVec = new Vector<Vertex>();
		Vector<Integer> nbrIdVec = new Vector<Integer>();
		
		// Preserve a neighbor if it is not to be removed
		for(int i = 0; i < neighbors.length; i ++){
			if(neighbors[i].id != id){
				nbrVec.add(neighbors[i]);
				nbrIdVec.add(id);
			}
		}
		
		// Update neighbors
		this.neighbors = new Vertex[nbrVec.size()];
		this.nbrsIds = new int[nbrVec.size()];
		for(int i = 0; i < this.neighbors.length; i ++){
			this.neighbors[i] = nbrVec.get(i);
			this.nbrsIds[i] = nbrIdVec.get(i);
		}
	}
	
	/**
	 * Do an weight update based on the number of neighbors. This is a simple 
	 * average - each neighbor holds equal amounts of information
	 */
	public void updateWeights(){
		weights = new double[neighbors.length];
		for(int i = 0; i < neighbors.length; i ++){
			weights[i] = 1./neighbors.length;
		}
	}

	/**
	 * Retrieves weight
	 * 
	 * @param id
	 * @return
	 */
	public double getWeight(int id){
		for(int i = 0; i < neighbors.length; i ++){
			if(neighbors[i].id == id){
				return weights[i];
			}
		}
		return 0;
	}
}
