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

import gurobi.GRB;
import gurobi.GRBEnv;
import gurobi.GRBException;
import gurobi.GRBLinExpr;
import gurobi.GRBModel;
import gurobi.GRBQuadExpr;
import gurobi.GRBVar;

import java.util.HashSet;
import java.util.Set;
import java.util.Vector;

/**
 * @author Jingjin Yu
 *
 * A solver class for the Quadratic Correlated Orienteering Problem
 * (QCOP). A comparative OP model is also implemented.
 * 
 */
public class QCOPSolver {

	/**
	 * The main solver function that treats the first node as the starting node.
	 * 
	 * @param graph The input graph, encoding the node
	 * @param loopMaxLength
	 * @param maxSingleHopDistance
	 * @param linearCost
	 * @param mipGap
	 * @return
	 * @throws GRBException
	 */
	public Object[] solve(Graph graph, double loopMaxLength,
			double maxSingleHopDistance, boolean linearCost, double mipGap)
			throws GRBException {
		// Create a GRB environment and a GRB model
		GRBEnv env = new GRBEnv();
		env.set(GRB.IntParam.OutputFlag, 0);
		env.set(GRB.DoubleParam.MIPGap, mipGap);
//		env.set(GRB.IntParam.Threads, 12);
		env.set(GRB.DoubleParam.TimeLimit, 2500.);
		GRBModel grbModel = new GRBModel(env);

		// Retrieve all of graph vertices
		Vertex[] vertices = graph.getVertices();

		// Create variables, first the (directed) edge variable x_{ij}'s
		GRBVar[][] xijs = new GRBVar[vertices.length][vertices.length];
		for (int i = 0; i < vertices.length; i++) {
			for (int j = 0; j < vertices.length; j++) {
				if (i != j
						&& graph.hasEdge(vertices[i].id, vertices[j].id,
								maxSingleHopDistance)) {
					xijs[i][j] = grbModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "v"
							+ vertices[i].id + ":" + vertices[j].id);
				}
			}
		}

		// Create variables indicating whether a vertex is used. Note that the
		// first vertex (v0) is always set to have a value 1
		GRBVar[] xis = new GRBVar[vertices.length];
		for (int i = 0; i < vertices.length; i++) {
			xis[i] = grbModel.addVar(i == 0 ? 1.0 : 0.0, 1.0, 0.0, GRB.BINARY,
					"v" + vertices[i].id);
		}

		// Integer variables for u_i's to prevent undesired loop
		GRBVar[] uis = new GRBVar[vertices.length];
		for (int i = 1; i < vertices.length; i++) {
			uis[i] = grbModel.addVar(2.0, vertices.length, 0.0, GRB.INTEGER,
					"u" + vertices[i].id);
		}
		grbModel.update();

		// Vertex inflow/outflow constraints; first vertex (vertex 0) must
		// be used once.
		{
			GRBLinExpr linExprOut = new GRBLinExpr();
			GRBLinExpr linExprIn = new GRBLinExpr();
			for (int i = 1; i < vertices.length; i++) {
				if (xijs[0][i] != null)
					linExprOut.addTerm(1., xijs[0][i]);
				if (xijs[i][0] != null)
					linExprIn.addTerm(1., xijs[i][0]);
			}

			// Outflow equals one (exact one outgoing edge must be used)
			grbModel.addConstr(linExprOut, GRB.EQUAL, 1, null);

			// Inflow equals one (exact one incoming edge must be used)
			grbModel.addConstr(linExprIn, GRB.EQUAL, 1, null);
		}

		// Vertex single use constraint for other vertices
		for (int k = 1; k < vertices.length; k++) {
			GRBLinExpr linExprOut = new GRBLinExpr();
			GRBLinExpr linExprIn = new GRBLinExpr();
			for (int i = 0; i < vertices.length; i++) {
				if (xijs[i][k] != null)
					linExprOut.addTerm(1., xijs[i][k]);
				if (xijs[k][i] != null)
					linExprIn.addTerm(1., xijs[k][i]);
			}

			// Outflow at most one
			// grbModel.addConstr(linExprOut, GRB.LESS_EQUAL, 1, null);

			// Inflow and outflow must be equal
			grbModel.addConstr(linExprOut, GRB.EQUAL, linExprIn, null);

			// If total outflow is one, then the vertex is used
			grbModel.addConstr(linExprOut, GRB.EQUAL, xis[k], null);
		}

		// Loop length constraint
		{
			GRBLinExpr linExpr = new GRBLinExpr();
			for (int i = 0; i < vertices.length; i++) {
				for (int j = 0; j < vertices.length; j++) {
					if (xijs[i][j] != null) {
						linExpr.addTerm(
							graph.edgeLengths[vertices[i].id][vertices[j].id],
							xijs[i][j]);
					}
				}
			}
			grbModel.addConstr(linExpr, GRB.LESS_EQUAL, loopMaxLength, null);
		}

		// Subtour prevention constraint
		for (int i = 1; i < vertices.length; i++) {
			for (int j = 1; j < vertices.length; j++) {
				if (xijs[i][j] != null) {
					GRBLinExpr linExpr = new GRBLinExpr();
					linExpr.addTerm(1., uis[i]);
					linExpr.addTerm(-1., uis[j]);
					linExpr.addTerm(vertices.length - 1., xijs[i][j]);

					grbModel.addConstr(linExpr, GRB.LESS_EQUAL,
							vertices.length - 2, null);
				}
			}
		}

		// Cost function. If linear cost, then it's regular OP; otherwise
		// QCOP is enforced
		if (linearCost == true) {
			GRBLinExpr linExpr = new GRBLinExpr();
			for (int i = 0; i < vertices.length; i++) {
				linExpr.addTerm(vertices[i].importance, xis[i]);
			}
			grbModel.setObjective(linExpr, GRB.MAXIMIZE);
		} else {
			GRBQuadExpr quadExpr = new GRBQuadExpr();
			for (int i = 0; i < vertices.length; i++) {
				quadExpr.addTerm(vertices[i].importance, xis[i]);
				for (int j = 0; j < vertices.length; j++) {
					if (i != j) {
						double weight = vertices[j].getWeight(vertices[i].id)
								* vertices[j].importance;
						if (weight > 0) {
							quadExpr.addTerm(weight, xis[i], xis[i]);
							quadExpr.addTerm(-weight, xis[i], xis[j]);
						}
					}
				}
			}
			grbModel.setObjective(quadExpr, GRB.MAXIMIZE);
		}

		// Update model before optimizing
		grbModel.update();
		// grbModel.write("C:\\Dev\\old-model.lp");

		// Run optimization
		grbModel.optimize();

		// Temp object array for holding return values
		Object optValue[] = new Object[2];

		// Check to make sure we have an optimal solution
		if (grbModel.get(GRB.IntAttr.Status) == GRB.Status.OPTIMAL) {
			optValue[0] = grbModel.get(GRB.DoubleAttr.ObjVal);
			// Retrieve the path, start from first vertex
			Vector<Vertex> vVec = new Vector<Vertex>();
			optValue[1] = vVec;
			vVec.add(vertices[0]);
			int currentIndex = 0;
			do {
				for (int i = 0; i < vertices.length; i++) {
					if (xijs[currentIndex][i] != null
						&& xijs[currentIndex][i].get(GRB.DoubleAttr.X) > 0.01) {
						if(i != 0){
							vVec.add(vertices[i]);
						}
						currentIndex = i;
						break;
					}
				}
			} while (currentIndex != 0);

			// for(int i = 0; i < vertices.length; i ++){
			// if(xis[i].get(GRB.DoubleAttr.X)>0.01){
			// System.out.print(vertices[i].id + " " );
			// }
			// }
			// System.out.println();
			// for(int i = 0; i < vertices.length; i ++){
			// for(int j = 0; j < vertices.length; j ++){
			// if(xijs[i][j]!= null
			// && xijs[i][j].get(GRB.DoubleAttr.X)>0.01){
			// System.out.print(vertices[i].id +
			// ":" +vertices[j].id + " ");
			// }
			// }
			// }
			
			if(linearCost == true){
				optValue[0] = computeQuadReward(vVec.toArray(new Vertex[0]));
			}
		}

		grbModel.dispose();
		return optValue;
	}

	/**
	 * Calls the solver with a specific starting node
	 * 
	 * @param graph
	 * @param loopMaxLength
	 * @param maxSingleHopDistance
	 * @param linearCost
	 * @param startVertextId
	 * @param mipGap
	 * @return
	 * @throws GRBException
	 */
	public static Object[] solveFullGraph(Graph graph, double loopMaxLength,
			double maxSingleHopDistance, boolean linearCost,
			int startVertextId, double mipGap) throws GRBException {
		// Change orders of vertices as needed; basically moves the vertex
		// with id = startVertextId to be the first vertex
		int size = graph.getVertices().length;
		int order[] = new int[size];
		for (int k = 0; k < size; k++) {
			order[k] = k;
		}
		order[0] = startVertextId;
		order[startVertextId] = 0;
		graph.setVertexOrder(order);

		// Inovoke the default solver
		Object optimalValue[] = new Object[size + 1];
		optimalValue[0] = new Double(0);

		QCOPSolver solver = new QCOPSolver();
		try {
			Object[] opt = solver.solve(graph, loopMaxLength,
					maxSingleHopDistance, linearCost, mipGap);
			optimalValue = opt;
		} catch (GRBException e) {
			e.printStackTrace();
		}

		// Restore vertex order
		for (int k = 0; k < size; k++) {
			order[k] = k;
		}
		graph.setVertexOrder(order);

		// Return
		return optimalValue;
	}

	/**
	 * Iteratively calls the solver with different vertices as the start vertex.
	 * This is the brute force version of solveFreeBase 
	 * 
	 * @param graph
	 * @param loopMaxLength
	 * @param maxSingleHopDistance
	 * @param linearCost
	 * @param mipGap
	 * @return
	 * @throws GRBException
	 */
	public static Object[] solveFullGraph(Graph graph, double loopMaxLength,
			double maxSingleHopDistance, boolean linearCost, double mipGap)
			throws GRBException {
		int size = graph.getVertices().length;
		Object optimalValue[] = new Object[size + 1];
		optimalValue[0] = new Double(0);
		for (int i = 0; i < size - 1; i++) {
			int order[] = new int[size];
			for (int k = 0; k < size; k++) {
				order[k] = k;
			}
			order[0] = i;
			order[i] = 0;
			graph.setVertexOrder(order);
			// System.out.println("Running with order:" + order[0] + " " +
			// order[size-1]);
			QCOPSolver solver = new QCOPSolver();
			try {
				Object[] opt = solver.solve(graph, loopMaxLength,
						maxSingleHopDistance, linearCost, mipGap);
				if (opt[0] != null
						&& (Double) (opt[0]) > (Double) (optimalValue[0])) {
					optimalValue = opt;
					// System.out.println("New optimal value: " +
					// optimalValue[0]);
				}
			} catch (GRBException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return optimalValue;
	}

	/**
	 * The main solver function that allows any vertex to be used as the base
	 * vertex. To do this we create a virtual source and split each actual node
	 * into two, with one having only outgoing edges and one has only incoming
	 * edges. The set baseIdSet specify which nodes may serve as base nodes.
	 * 
	 * @param graph
	 * @param baseIdSet
	 * @param loopMaxLength
	 * @param maxSingleHopDistance
	 * @param linearCost
	 * @param mipGap
	 * @return
	 * @throws GRBException
	 */
	public static Object[] solveFreeBase(Graph graph, 
			Set<Integer> baseIdSet,	double loopMaxLength, 
			double maxSingleHopDistance, boolean linearCost, double mipGap) 
			throws GRBException {
		// Create a GRB environment and a GRB model
		GRBEnv env = new GRBEnv();
		env.set(GRB.IntParam.OutputFlag, 1);
		env.set(GRB.DoubleParam.MIPGap, mipGap);
		env.set(GRB.DoubleParam.TimeLimit, 2500.);
		GRBModel grbModel = new GRBModel(env);

		// Retrieve all of graph vertices
		Vertex[] vertices = graph.getVertices();

		// The edge variables from/to the virtual node
		GRBVar[] vOut = new GRBVar[vertices.length];
		GRBVar[] vIn = new GRBVar[vertices.length];
		GRBVar[] vInOut = new GRBVar[vertices.length];
		GRBVar[] vOutIn = new GRBVar[vertices.length];
		for (int i = 0; i < vOut.length; i++) {
			if (baseIdSet.contains(vertices[i].id)) {
				vOut[i] = grbModel.addVar(0.0, 1.0, 0.0,
						GRB.BINARY, "o" + vertices[i].id);

				vIn[i] = grbModel.addVar(0.0, 1.0, 0.0,
						GRB.BINARY, "i" + vertices[i].id);

				vInOut[i] = grbModel.addVar(0.0, 1.0, 0.0,
						GRB.BINARY, "io" + vertices[i].id);
				
				vOutIn[i] = grbModel.addVar(0.0, 1.0, 0.0,
						GRB.BINARY, "oi" + vertices[i].id);
			}
		}

		// First the (directed) edge variable x_{ij}'s
		GRBVar[][] xijs = new GRBVar[vertices.length][vertices.length];
		for (int i = 0; i < vertices.length; i++) {
			for (int j = 0; j < vertices.length; j++) {
				if (i != j
						&& graph.hasEdge(vertices[i].id, vertices[j].id,
								maxSingleHopDistance)) {
					xijs[i][j] = grbModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "v"
							+ vertices[i].id + ":" + vertices[j].id);
				}
			}
		}

		// Create variables indicating whether a vertex is used
		GRBVar[] xis = new GRBVar[vertices.length];
		for (int i = 0; i < vertices.length; i++) {
			xis[i] = grbModel.addVar(0.0, 1.0, 0.0, GRB.BINARY,
					"v" + vertices[i].id);
		}

		// Integer variables for u_i's to prevent undesired loop
		GRBVar[] uis = new GRBVar[vertices.length];
		for (int i = 0; i < vertices.length; i++) {
			uis[i] = grbModel.addVar(1.0, vertices.length, 0.0, GRB.INTEGER,
					"u" + vertices[i].id);
		}
		grbModel.update();

		// Vertex inflow/outflow constraints at the virtual vertex
		{
			GRBLinExpr linExprOut = new GRBLinExpr();
			GRBLinExpr linExprIn = new GRBLinExpr();
			for (int i = 0; i < vertices.length; i++) {
				if (vOut[i] != null)
					linExprOut.addTerm(1., vOut[i]);
				if (vIn[i] != null)
					linExprIn.addTerm(1., vIn[i]);
			}

			// Outflow equals one (exact one outgoing edge must be used)
			grbModel.addConstr(linExprOut, GRB.EQUAL, 1, null);

			// Inflow equals one (exact one incoming edge must be used)
			grbModel.addConstr(linExprIn, GRB.EQUAL, 1, null);
		}

		// Vertex single use constraint for other vertices
		for (int k = 0; k < vertices.length; k++) {
			GRBLinExpr linExprOut = new GRBLinExpr();
			GRBLinExpr linExprIn = new GRBLinExpr();
			for (int i = 0; i < vertices.length; i++) {
				if (xijs[k][i] != null)
					linExprOut.addTerm(1., xijs[k][i]);
				if (xijs[i][k] != null)
					linExprIn.addTerm(1., xijs[i][k]);
			}

			// If total outflow is one, then the vertex is used
			grbModel.addConstr(linExprOut, GRB.EQUAL, xis[k], null);

			// Inflow and outflow must be equal
			grbModel.addConstr(linExprOut, GRB.EQUAL, linExprIn, null);

			// Base vertex needs some extra care
			if (vOut[k] != null) {
				linExprOut.addTerm(1., vOutIn[k]);
				linExprOut.addTerm(-1., vInOut[k]);
				linExprOut.addTerm(-1., vOut[k]);
				grbModel.addConstr(linExprOut, GRB.EQUAL, 0, null);
				
				linExprIn.addTerm(1., vOutIn[k]);
				linExprIn.addTerm(-1., vInOut[k]);
				linExprIn.addTerm(-1., vIn[k]);
				grbModel.addConstr(linExprIn, GRB.EQUAL, 0, null);
			}
		}

		// Loop length constraint
		{
			GRBLinExpr linExpr = new GRBLinExpr();
			for (int i = 0; i < vertices.length; i++) {
				for (int j = 0; j < vertices.length; j++) {
					if (xijs[i][j] != null) {
						linExpr.addTerm(
							graph.edgeLengths[vertices[i].id][vertices[j].id],
							xijs[i][j]);
					}
				}
			}
			grbModel.addConstr(linExpr, GRB.LESS_EQUAL, loopMaxLength, null);
		}

		// Subtour prevention constraint
		for (int i = 0; i < vertices.length; i++) {
			for (int j = 0; j < vertices.length; j++) {
				if (i != j && xijs[i][j] != null) {
					GRBLinExpr linExpr = new GRBLinExpr();
					linExpr.addTerm(1., uis[i]);
					linExpr.addTerm(-1., uis[j]);
					linExpr.addTerm(vertices.length, xijs[i][j]);

					if(vOut[i] != null){
						linExpr.addTerm(vertices.length, vInOut[i]);
						grbModel.addConstr(linExpr, GRB.LESS_EQUAL,
								vertices.length*2 - 1, null);
						
					}
					else if(vOut[j] != null){
						linExpr.addTerm(vertices.length, vInOut[j]);
						grbModel.addConstr(linExpr, GRB.LESS_EQUAL,
								vertices.length*2 - 1, null);
					}
					else{
						grbModel.addConstr(linExpr, GRB.LESS_EQUAL,
							vertices.length - 1, null);
					}
				}
			}
		}

		// Cost function. If linear cost, then it's regular OP; otherwise
		// QCOP is enforced
		if (linearCost == true) {
			GRBLinExpr linExpr = new GRBLinExpr();
			for (int i = 0; i < vertices.length; i++) {
				linExpr.addTerm(vertices[i].importance, xis[i]);
			}
			grbModel.setObjective(linExpr, GRB.MAXIMIZE);
		} else {
			GRBQuadExpr quadExpr = new GRBQuadExpr();
			for (int i = 0; i < vertices.length; i++) {
				quadExpr.addTerm(vertices[i].importance, xis[i]);
				for (int j = 0; j < vertices.length; j++) {
					if (i != j) {
						double weight = vertices[j].getWeight(vertices[i].id)
								* vertices[j].importance;
						if (weight > 0) {
							quadExpr.addTerm(weight, xis[i], xis[i]);
							quadExpr.addTerm(-weight, xis[i], xis[j]);
						}
					}
				}
			}
			grbModel.setObjective(quadExpr, GRB.MAXIMIZE);
		}

		// Update model before optimizing
		grbModel.update();
		// grbModel.write("C:\\Dev\\model.lp");

		// Run optimization
		grbModel.optimize();

		// Temp object array for holding return values
		Object optValue[] = new Object[2];

		// Check to make sure we have an optimal solution
		if (grbModel.get(GRB.IntAttr.Status) == GRB.Status.OPTIMAL) {
			optValue[0] = grbModel.get(GRB.DoubleAttr.ObjVal);
			// Retrieve the path, start from first vertex
			Vector<Vertex> vVec = new Vector<Vertex>();
			optValue[1] = vVec;
			
			// First find which start node is used
			int firstIndex = 0;
			for(int i = 0; i < vOut.length; i++ ){
				if(vOut[i] != null){
					if(vOut[i].get(GRB.DoubleAttr.X) > 0.01){
						vVec.add(vertices[i]);
						firstIndex = i;
						break;
					}
				}
			}

//			// For debug
//			System.out.println();
//			for(int i = 0; i < xis.length; i++ ){
//				if(xis[i].get(GRB.DoubleAttr.X) > 0.01){
//					System.out.print(" " + i);
//					System.out.print(":"+ uis[i].get(GRB.DoubleAttr.X));
//				}
//			}
//			System.out.println();

			// Then find the tour path
			int currentIndex = firstIndex;
			do {
				for (int i = 0; i < vertices.length; i++) {
					if (xijs[currentIndex][i] != null
						&& xijs[currentIndex][i].get(GRB.DoubleAttr.X) > 0.01) {
						if(i != firstIndex){
							vVec.add(vertices[i]);
						}
						currentIndex = i;
						break;
					}
				}
			} while (currentIndex != firstIndex);

			if(linearCost == true){
				optValue[0] = computeQuadReward(vVec.toArray(new Vertex[0]));
			}
		}

		grbModel.dispose();
		return optValue;
	}

	public static double computeQuadReward(Vertex[] tour){
		// Collect all viisted vertices
		Set<Integer> tourVertexSet = new HashSet<Integer>();
		for(int i = 0; i < tour.length; i++){
			tourVertexSet.add(tour[i].id);
		}
		
		// Compute the reward
		double reward = 0;
		for(int i = 0; i < tour.length; i++){
			reward += tour[i].importance;
			for(Vertex nv:tour[i].neighbors){
				if(!tourVertexSet.contains(nv.id)){
					reward += nv.importance*nv.getWeight(tour[i].id);
				}
			}
		}
		return reward;
	}
	
	public static void main(String argv[]) {
		int n = 5;
		
		// Testing the fixed based method
		Graph g = Graph.getnxnGrid(n);
		// g.randomize();

		Set<Integer> hs = new HashSet<Integer>();
		Vertex vss[] = g.getVertices();
		for(Vertex v:vss){hs.add(v.id);}
		//hs.clear();
		//hs.add(1);
		
		try {
			Object sol[] = solveFreeBase(g, hs, 12, 20, true, 0);
			if (sol[0] != null) {
				Vertex[] vs = ((Vector<Vertex>) (sol[1]))
						.toArray(new Vertex[0]);
				System.out.print("" + computeQuadReward(vs) + ", "
						+ (Double) sol[0] + "   ");
				for (int i = 0; i < vs.length; i++) {
					System.out.print(vs[i].id + " ");
				}
			}
		} catch (GRBException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// Testing the floating base method
		System.out.println("\nSecond method");
		// g = Graph.getnxnGrid(n);
		try {
			
			Object sol[] = solveFreeBase(g, hs, 12, 20, false, 0);
			System.out.print((Double) sol[0] + "   ");
			if (sol[0] != null) {
				Vertex[] vs = ((Vector<Vertex>) (sol[1]))
						.toArray(new Vertex[0]);
				for (int i = 0; i < vs.length; i++) {
					System.out.print(vs[i].id + " ");
				}
			}
		} catch (GRBException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
