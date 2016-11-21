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
import java.util.Set;
import java.util.Vector;

import gurobi.GRB;
import gurobi.GRBEnv;
import gurobi.GRBException;
import gurobi.GRBLinExpr;
import gurobi.GRBModel;
import gurobi.GRBQuadExpr;
import gurobi.GRBVar;

/**
 * 
 * @author Jingjin
 *
 * Solving QCOP with multi-tour support
 * 
 */
public class QCOPMultiSolver {

	// Offset for creating
	public static long OFFSET = 100000L;

	// GRB model
	GRBModel grbModel = null;

	public Object[] solve(Graph g, int[] svs, double[] loopMaxLength,
			double cutOffDistance, boolean linearCostOnly, double gap)
			throws GRBException {
		// Set up GRB environment
		GRBEnv env = new GRBEnv();
		env.set(GRB.IntParam.OutputFlag, 0);
		env.set(GRB.DoubleParam.MIPGap, gap);
		env.set(GRB.DoubleParam.TimeLimit, 2500.);
		grbModel = new GRBModel(env);

		// Retrieve all of g's vertices
		Vertex[] vertices = g.getVertices();

		// Create variables, first x_{ija}'s
		GRBVar[][][] xijas = 
				new GRBVar[vertices.length][vertices.length][svs.length];
		for (int i = 0; i < vertices.length; i++) {
			for (int j = 0; j < vertices.length; j++) {
				for (int a = 0; a < svs.length; a++) {
					if (i != j
							&& g.hasEdge(vertices[i].id, vertices[j].id,
									cutOffDistance)) {
						xijas[i][j][a] = grbModel.addVar(0.0, 1.0, 0.0,
								GRB.BINARY, "v" + vertices[i].id + ":"
										+ vertices[j].id + ">" + a);
					}

				}
			}
		}
		// Then variables indicating whether a vertex is used
		GRBVar[] xis = new GRBVar[vertices.length];
		for (int i = 0; i < vertices.length; i++) {
			xis[i] = grbModel.addVar(0.0, 1.0, 0.0, GRB.BINARY,
					"v" + vertices[i].id);
		}

		// Then u_{ia}'s
		GRBVar[][] uias = new GRBVar[vertices.length][svs.length];
		for (int i = 0; i < vertices.length; i++) {
			for (int a = 0; a < svs.length; a++) {
				uias[i][a] = grbModel.addVar(2.0, vertices.length, 0.0,
						GRB.INTEGER, "u" + vertices[i].id + ">" + a);
			}
		}
		
		// Variable to allow loop at start vertex
		GRBVar[] sls = new GRBVar[svs.length];
		for(int i = 0; i < sls.length; i++){
			sls[i] = grbModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "s" + svs[i]);
		}
		
		grbModel.update();

		// Start from vertex constraint
		{
			for (int a = 0; a < svs.length; a++) {
				GRBLinExpr linExprF = new GRBLinExpr();
				GRBLinExpr linExprL = new GRBLinExpr();
				for (int i = 0; i < vertices.length; i++) {
					int startVertexId = svs[a];
					if (xijas[startVertexId][i][a] != null)
						linExprF.addTerm(1., xijas[startVertexId][i][a]);
					if (xijas[i][startVertexId][a] != null)
						linExprL.addTerm(1., xijas[i][startVertexId][a]);
				}
				// Allow self loop
				linExprF.addTerm(1., sls[a]);
				linExprL.addTerm(1., sls[a]);
				
				// Add constraints to model
				grbModel.addConstr(linExprF, GRB.EQUAL, 1, null);
				// grbModel.addConstr(xis[svs[a]], GRB.EQUAL, 1, null);
				// grbModel.addConstr(sls[a], GRB.EQUAL, 1, null);
				// grbModel.addConstr(linExprL, GRB.EQUAL, 1, null);
			}
		}

		// Vertex single use constraint
		for (int k = 0; k < vertices.length; k++) {
			GRBLinExpr linExprF = new GRBLinExpr();
			GRBLinExpr linExprL = new GRBLinExpr();
//			for (int i = 0; i < vertices.length; i++) {
//				for (int a = 0; a < svs.length; a++) {
//					if (xijas[i][k][a] != null)
//						linExprF.addTerm(1., xijas[i][k][a]);
//					if (xijas[k][i][a] != null)
//						linExprL.addTerm(1., xijas[k][i][a]);
//				}
//			}
//			grbModel.addConstr(linExprF, GRB.LESS_EQUAL, 1, null);
//			grbModel.addConstr(linExprL, GRB.LESS_EQUAL, 1, null);

			for (int a = 0; a < svs.length; a++) {
				linExprF = new GRBLinExpr();
				linExprL = new GRBLinExpr();
				for (int i = 0; i < vertices.length; i++) {
					if (xijas[i][k][a] != null)
						linExprF.addTerm(1., xijas[i][k][a]);
					if (xijas[k][i][a] != null)
						linExprL.addTerm(1., xijas[k][i][a]);
				}
				grbModel.addConstr(linExprF, GRB.EQUAL, linExprL, null);
			}

			GRBLinExpr linExpr = new GRBLinExpr();
			for (int a = 0; a < svs.length; a++) {
				for (int i = 0; i < vertices.length; i++) {
					if (xijas[k][i][a] != null)
						linExpr.addTerm(1., xijas[k][i][a]);
				}
				if(k == svs[a]){
					linExpr.addTerm(1., sls[a]);
				}
			}
			grbModel.addConstr(linExpr, GRB.EQUAL, xis[k], null);
		}

		// Total length constraint
		for (int a = 0; a < svs.length; a++) {
			GRBLinExpr linExpr = new GRBLinExpr();
			for (int i = 0; i < vertices.length; i++) {
				for (int j = 0; j < vertices.length; j++) {
					if (xijas[i][j][a] != null)
						linExpr.addTerm(
								g.edgeLengths[vertices[i].id][vertices[j].id],
								xijas[i][j][a]);
				}
			}
			grbModel.addConstr(linExpr, GRB.LESS_EQUAL, loopMaxLength[a], null);
		}

		// Subtour restriction constraint
		for (int a = 0; a < svs.length; a++) {
			int startVertexId = svs[a];
			for (int i = 0; i < vertices.length; i++) {
				for (int j = 0; j < vertices.length; j++) {
					if (xijas[i][j][a] != null && i != startVertexId
							&& j != startVertexId) {
						GRBLinExpr linExpr = new GRBLinExpr();
						linExpr.addTerm(1., uias[i][a]);
						linExpr.addTerm(-1., uias[j][a]);
						linExpr.addTerm(vertices.length - 1., xijas[i][j][a]);
						grbModel.addConstr(linExpr, GRB.LESS_EQUAL,
								vertices.length - 2, null);
					}
				}
			}
		}

		if (linearCostOnly == true) {
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
						double weight = vertices[j].importance*
								vertices[j].getWeight(vertices[i].id);
						if (weight > 0) {
							quadExpr.addTerm(weight, xis[i], xis[i]);
							quadExpr.addTerm(-weight, xis[i], xis[j]);
						}
					}
				}
			}
			grbModel.setObjective(quadExpr, GRB.MAXIMIZE);
		}

		grbModel.update();
		// grbModel.write("C:\\Users\\Jingjin\\Desktop\\model.lp");
		grbModel.optimize();
		Object optValue[] = new Object[1 + svs.length];
		int status = grbModel.get(GRB.IntAttr.Status);
		if (status == GRB.Status.OPTIMAL) {
			optValue[0] = grbModel.get(GRB.DoubleAttr.ObjVal);
			// Retrieve the path, start from first vertex
			Vector<Vertex> avVec = new Vector<Vertex>();

			for (int a = 0; a < svs.length; a++) {
				int startVertexId = svs[a];
				Vector<Vertex> vVec = new Vector<Vertex>();
				optValue[a + 1] = vVec;
				avVec.add(vertices[startVertexId]);
				vVec.add(vertices[startVertexId]);
				int currentIndex = startVertexId;
				do {
					for (int i = 0; i < vertices.length; i++) {
						if (xijas[currentIndex][i][a] != null
								&& xijas[currentIndex][i][a]
										.get(GRB.DoubleAttr.X) > 0.01) {
							vVec.add(vertices[i]);
							if(vertices[i].id != startVertexId)
								avVec.add(vertices[i]);
							currentIndex = i;
							break;
						}
					}
				} while (currentIndex != startVertexId);
			}
			
			if(linearCostOnly == true){
				optValue[0] = computeQuadReward(avVec.toArray(new Vertex[0]));
			}

			// for(int i = 0; i < vertices.length; i ++){
			// if(xis[i].get(GRB.DoubleAttr.X)>0.01){
			// System.out.print(vertices[i].id + " " );
			// }
			// }
			// System.out.println();
			// for(int i = 0; i < vertices.length; i ++){
			// for(int j = 0; j < vertices.length; j ++){
			// if(xijs[i][j]!= null && xijs[i][j].get(GRB.DoubleAttr.X)>0.01){
			// System.out.print(vertices[i].id + ":" +vertices[j].id + " ");
			// }
			// }
			// }
		}

		grbModel.dispose();
		return optValue;
	}

	// public static Object[] solveFullGraph(Graph g, int[] svs, double
	// loopMaxLength, double cutOffDistance, boolean linearCostOnly) throws
	// GRBException{
	// int size = g.getVertices().length;
	// Object optimalValue[] = new Object[size+1];
	// optimalValue[0] = new Double(0);
	// for(int i = 0; i < size-1; i ++){
	// int order[] = new int[size];
	// for(int k = 0; k < size; k ++){
	// order[k] = k;
	// }
	// order[0] = i;
	// order[i] = 0;
	// g.setVertexOrder(order);
	// // System.out.println("Running with order:" + order[0] + " " +
	// order[size-1]);
	// QTOPLoopMultiSolver solver = new QTOPLoopMultiSolver();
	// try {
	// Object[] opt = solver.solve(g, svs, loopMaxLength, cutOffDistance,
	// false);
	// if(opt[0]!=null && (Double)(opt[0]) > (Double)(optimalValue[0])){
	// optimalValue = opt;
	// // System.out.println("New optimal value: " + optimalValue[0]);
	// }
	// } catch (GRBException e) {
	// // TODO Auto-generated catch block
	// e.printStackTrace();
	// }
	// }
	// return optimalValue;
	// }

	public static Object[] solveFullGraphMulti(Graph g, int[] svs,
			double[] loopMaxLength, double cutOffDistance,
			boolean linearCostOnly, double gap) throws GRBException {
		int size = g.getVertices().length;
		Object optimalValue[] = new Object[size + 1];
		optimalValue[0] = new Double(0);

		// System.out.println("Running with order:" + order[0] + " " +
		// order[size-1]);
		QCOPMultiSolver solver = new QCOPMultiSolver();
		try {
			Object[] opt = solver.solve(g, svs, loopMaxLength, cutOffDistance,
					linearCostOnly, gap);
			optimalValue = opt;
		} catch (GRBException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		return optimalValue;
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
		int n = 4;
		Graph g = Graph.getnxnGrid(n);
		// Graph g = Graph.getTestCaseGraph2();
		try {
			Object sol[] = solveFullGraphMulti(g, new int[] { 1, 14 },
					new double[] { 2, 4 }, 10, false, 0);
			// Object sol[] = solveFullGraphMulti(g, new int[]{2, 7, 16}, 14,
			// 10, false);
			System.out.print(sol[0] + "   ");
			System.out.println();
			if (sol[0] != null) {
				for (int a = 0; a < sol.length - 1; a++) {
					double distance = 0;
					Vertex[] vs = ((Vector<Vertex>) (sol[a + 1]))
							.toArray(new Vertex[0]);
					for (int i = 0; i < vs.length; i++) {
						System.out.print(vs[i].id + " ");
						if (i < vs.length - 1) {
							distance += g.edgeLengths[vs[i].id][vs[i + 1].id];
						}
					}
					System.out.println(" - " + distance);
				}
			}
		} catch (GRBException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// int n = 3;
		// Graph g = Graph.getnxnGrid(n);
		// try {
		// g.setVertexOrder(new int[]{5,0,1,2,3,4,6,8,7});
		// BudgetedLoopQPSolverSingleShotFlat solver = new
		// BudgetedLoopQPSolverSingleShotFlat();
		// Object sol[] = solver.solve(g, 5, 3);
		// System.out.print((Double)sol[0] + "   ");
		// for(int i = 1;i <sol.length;i++){
		// if(sol[i] != null)
		// System.out.print(((Vertex)sol[i]).id + " ");
		// }
		// } catch (GRBException e) {
		// // TODO Auto-generated catch block
		// e.printStackTrace();
		// }

		//
		// Graph g = Graph.getTestCaseGraph();
		// BudgetedLoopQPSolverSingleShotFlat solver = new
		// BudgetedLoopQPSolverSingleShotFlat();
		// try {
		// solver.solve(g, 4, 8, 200);
		// } catch (GRBException e) {
		// // TODO Auto-generated catch block
		// e.printStackTrace();
		// }

	}
}
