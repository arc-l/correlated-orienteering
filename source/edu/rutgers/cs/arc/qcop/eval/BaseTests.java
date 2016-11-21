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


package edu.rutgers.cs.arc.qcop.eval;

import edu.rutgers.cs.arc.qcop.Graph;
import edu.rutgers.cs.arc.qcop.QCOPSolver;
import edu.rutgers.cs.arc.qcop.QCOPMultiSolver;
import edu.rutgers.cs.arc.qcop.Vertex;
import gurobi.GRBException;

import java.util.Vector;

/**
 * 
 * @author Jingjin
 * 
 *         The basic set of tests on grids and irregular graphs with weights
 *         generated based on the number of neighbors.
 *
 */
public class BaseTests {
	public static double lastSystemTime = System.currentTimeMillis();

	public static double elapsedTime() {
		double time = (System.currentTimeMillis() - lastSystemTime) / 1000.;
		lastSystemTime = System.currentTimeMillis();
		return time;
	}

	public static double resetTime() {
		double time = (System.currentTimeMillis() - lastSystemTime) / 1000.;
		lastSystemTime = System.currentTimeMillis();
		return time;
	}

	public static void testSingleRobotRandomPerformance() throws GRBException {
		System.out.println("Testing single robot on randomized grids...");
		int[] gridSizes = new int[] { 4, 5, 6 };
		double maxSingleLegLength = 15;
		int budgetIncrements = 5;
		int startVertexId = 1;
		for (int s = 0; s < gridSizes.length; s++) {
			for (int l = 0; l < budgetIncrements; l++) {
				for (int i = 0; i < 10; i++) {
					Graph g = Graph.getnxnGrid(gridSizes[s]);
					g.randomizeVertexLocations(0.5);
					
					double budget = gridSizes[s] * 4.0 * (l + 1)
							/ budgetIncrements;
					
					Object sol[] = QCOPSolver.solveFullGraph(g, budget,
							maxSingleLegLength, false, startVertexId, 0);
					if (sol[0] == null){
						System.out.println();
						resetTime();
					}
					else {
						System.out.print("Running: |V|="
								+ (gridSizes[s] * gridSizes[s]) + ", Budget="
								+ budget + ", Max leg length="
								+ maxSingleLegLength + ", SV=" + startVertexId
								+ ", Computation time: " + elapsedTime() + " ");
						System.out.println(getOutput(sol, g));
					}
					
//					sol = QCOPSolver.solveFullGraph(g, budget,
//							maxSingleLegLength, false, startVertexId, 0.2);
//					if (sol[0] == null){
//						System.out.println();
//						resetTime();
//					}
//					else {
//						System.out.print("Running: |V|="
//								+ (gridSizes[s] * gridSizes[s]) + ", Budget="
//								+ budget + ", Max leg length="
//								+ maxSingleLegLength + ", SV=" + startVertexId
//								+ ", Computation time: " + elapsedTime() + " ");
//						System.out.println(getOutput(sol, g));
//					}
				}
			}
		}
	}

	public static void testSinglePurturbed(double gap) throws GRBException {
		System.out.println("Testing single robot on randomized grids with gap: "
				+ gap);
		int[] gridSizes = new int[] { 5, 6, 7, 8};
		double maxSingleLegLength = 15;
		int budgetIncrements = 5;
		int startVertexId = 1;
		for (int s = 0; s < gridSizes.length; s++) {
			if(gridSizes[s] < 8) continue;
			for (int l = 0; l < budgetIncrements; l++) {
				for (int i = 0; i < 5; i++) {
					Graph g = Graph.getnxmGrid(gridSizes[s], gridSizes[s] + s + 1);
//					g.randomizeVertexLocations(0.75);
//					g.randomizeEdges(0.15, 0.8);
//					g.randomizeVertexImportance(0.4);
					g.randomizeVertexLocations(0.5);
					g.randomizeEdges(0.5, 0.5);
					g.randomizeVertexImportance(0.5);
					
					double budget = (gridSizes[s] + s + 1) * 5 * (l + 1)
							/ budgetIncrements;
					
					Object sol[] = QCOPSolver.solveFullGraph(g, budget,
							maxSingleLegLength, false, startVertexId, gap);
					if (sol[0] == null){
						System.out.println();
						resetTime();
					}
					else {
						System.out.print("Running: |V|="
								+ (gridSizes[s] * (gridSizes[s] + s + 1)) 
								+ ", Budget="
								+ budget + ", Max leg length="
								+ maxSingleLegLength + ", SV=" + startVertexId
								+ ", Computation time: " + elapsedTime() + " ");
						System.out.println(getOutput(sol, g));
					}
					
				}
			}
		}
	}

	public static void testSinglePurturbedSquare(double gap) throws GRBException {
		System.out.println("Testing single robot on randomized grids with gap: "
				+ gap);
		int[] gridSizes = new int[] { 4, 5, 6};
		double maxSingleLegLength = 15;
		int budgetIncrements = 5;
		int startVertexId = 1;
		for (int s = 0; s < gridSizes.length; s++) {
			for (int l = 0; l < budgetIncrements; l++) {
				for (int i = 0; i < 5; i++) {
					Graph g = Graph.getnxmGrid(gridSizes[s], gridSizes[s]);
//					g.randomizeVertexLocations(0.75);
//					g.randomizeEdges(0.15, 0.8);
//					g.randomizeVertexImportance(0.4);
					g.randomizeVertexLocations(0.5);
					g.randomizeEdges(0.5, 0.5);
					g.randomizeVertexImportance(0.5);
					
					double budget = gridSizes[s]* 4. * (l + 1)
							/ budgetIncrements;
					
					Object sol[] = QCOPSolver.solveFullGraph(g, budget,
							maxSingleLegLength, false, startVertexId, gap);
					if (sol[0] == null){
						System.out.println();
						resetTime();
					}
					else {
						System.out.print("Running: |V|="
								+ (gridSizes[s] * (gridSizes[s])) 
								+ ", Budget="
								+ budget + ", Max leg length="
								+ maxSingleLegLength + ", SV=" + startVertexId
								+ ", Computation time: " + elapsedTime() + " ");
						System.out.println(getOutput(sol, g));
					}
					
				}
			}
		}
	}

	public static void testSingleRobotRegularSubopt(boolean linear, double gap) 
			throws GRBException {
		System.out.println("Testing single robot on randomized grids, " 
			+ (linear?"OP":"COP") + ", GAP: " + gap);
		int[] gridSizes = new int[] { 6, 7, 8, 9, 10, 11, 12, 13, 14 }; // 12
		double maxSingleLegLength = 50;
		int budgetIncrements = 5;
		int startVertexId = 1;
		for (int s = 0; s < gridSizes.length; s++) {
			Graph g = Graph.getnxnGrid(gridSizes[s]);
			// g.randomize();
			for (int l = 0; l < budgetIncrements; l++) {
				double budget = gridSizes[s] * 5 * (l + 1)
						/ (budgetIncrements * (gridSizes[s] < 15 ? 0.9 : 0.5));
				Object sol[] = QCOPSolver.solveFullGraph(g, budget,
						maxSingleLegLength, linear, startVertexId, gap);
				if (sol[0] == null){
					System.out.println();
					resetTime();
				}					
				else {
					System.out.print("Running: |V|="
							+ (gridSizes[s] * gridSizes[s]) + ", Budget="
							+ budget + ", Max leg length=" + maxSingleLegLength
							+ ", SV=" + startVertexId + ", Computation time: "
							+ elapsedTime() + " ");
					System.out.println(getOutput(sol, g));
				}
			}
		}
	}

	public static void testTwoRobotsRandomPerformanceSubopt()
			throws GRBException {
		System.out
				.println("Testing two robots, any time, on randomized grids...");
		int[] gridSizes = new int[] { 4, 5, 6, 7 };
		int svs[] = new int[2];
		double maxSingleLegLength = 50;
		int budgetIncrements = 5;
		for (int s = 0; s < gridSizes.length; s++) {
			for (int i = 0; i < 10; i++) {
				Graph g = Graph.getnxnGrid(gridSizes[s]);
				svs[0] = 1;
				svs[1] = gridSizes[s] * gridSizes[s] - 2;
				g.randomizeVertexLocations(0.5);
				for (int l = 1; l < budgetIncrements; l++) {
					double budget[] = new double[] {
							gridSizes[s] * 2 * (l + 1)
									/ (budgetIncrements * 1.0),
							gridSizes[s] * 2 * (l + 1)
									/ (budgetIncrements * 1.0) };
					Object sol[] = QCOPMultiSolver.solveFullGraphMulti(g, svs,
							budget, maxSingleLegLength, false, 0.2);
					if (sol[0] == null
							|| ((Double) sol[0]).doubleValue() <= 0.0)
					{
						System.out.println();
						resetTime();
					}
					else {
						System.out.print("Running: |V|="
								+ (gridSizes[s] * gridSizes[s]) + ", Budget="
								+ budget[0] + ", Max leg length="
								+ maxSingleLegLength + ", Computation time: "
								+ elapsedTime() + " ");
						System.out.println(getOutput(sol, g));
					}
				}
			}
		}
	}

	public static void testThreeRobotsRandomPerformanceSubopt()
			throws GRBException {
		System.out
				.println("Testing three robots, any time, on randomized grids...");
		int[] gridSizes = new int[] { 4, 5, 6, 7 };
		int svs[] = new int[3];
		double maxSingleLegLength = 50;
		int budgetIncrements = 5;
		for (int s = 0; s < gridSizes.length; s++) {
			for (int i = 0; i < 10; i++) {
				Graph g = Graph.getnxnGrid(gridSizes[s]);
				svs[0] = 1;
				svs[1] = gridSizes[s] * gridSizes[s] - gridSizes[s] - 1;
				svs[2] = gridSizes[s] * gridSizes[s] - 2 * gridSizes[s];
				g.randomizeVertexLocations(0.5);
				for (int l = 1; l < budgetIncrements; l++) {
					double budget[] = new double[] {
							gridSizes[s] * 2 * (l + 1)
									/ (budgetIncrements * 1.5),
							gridSizes[s] * 2 * (l + 1)
									/ (budgetIncrements * 1.5),
							gridSizes[s] * 2 * (l + 1)
									/ (budgetIncrements * 1.5) };
					// double budget = gridSizes[s]*2*(l +
					// 1)/(budgetIncrements*1.5);
					Object sol[] = QCOPMultiSolver.solveFullGraphMulti(g, svs,
							budget, maxSingleLegLength, false, 0.2);
					if (sol[0] == null
							|| ((Double) sol[0]).doubleValue() <= 0.0)
					{
						System.out.println();
						resetTime();
					}
					else {
						System.out.print("Running: |V|="
								+ (gridSizes[s] * gridSizes[s]) + ", Budget="
								+ budget[0] + ", Max leg length="
								+ maxSingleLegLength + ", Computation time: "
								+ elapsedTime() + " ");
						System.out.println(getOutput(sol, g));
					}
				}
			}
		}
	}

	public static void testThreeRobotsRegularSubopt() throws GRBException {
		System.out
				.println("Testing three robots, any time, on randomized grids...");
		int[] gridSizes = new int[] { 4, 5, 6};
		int svs[] = new int[3];
		double maxSingleLegLength = 2;
		int budgetIncrements = 5;
		for (int s = 0; s < gridSizes.length; s++) {
			Graph g = Graph.getnxnGrid(gridSizes[s]);
			svs[0] = 1;
			svs[1] = gridSizes[s] * gridSizes[s] - gridSizes[s] - 1;
			svs[2] = gridSizes[s] * gridSizes[s] - 2 * gridSizes[s];
			// g.randomize();
			for (int l = 1; l < budgetIncrements; l++) {
				double budget[] = new double[] {
						gridSizes[s] * 2 * (l + 1) / (budgetIncrements * 1.5),
						gridSizes[s] * 2 * (l + 1) / (budgetIncrements * 1.5),
						gridSizes[s] * 2 * (l + 1) / (budgetIncrements * 1.5) };
				// double budget = gridSizes[s]*2*(l +
				// 1)/(budgetIncrements*1.5);
				Object sol[] = QCOPMultiSolver.solveFullGraphMulti(g, svs,
						budget, maxSingleLegLength, false, 0.2);
				if (sol[0] == null || ((Double) sol[0]).doubleValue() <= 0.0)
				{
					System.out.println();
					resetTime();
				}
				else {
					System.out.print("Running: |V|="
							+ (gridSizes[s] * gridSizes[s]) + ", Budget="
							+ budget[0] + ", Max leg length="
							+ maxSingleLegLength + ", Computation time: "
							+ elapsedTime() + " ");
					System.out.println(getOutput(sol, g));
				}
			}
		}
	}

	public static void testSingleRobotRegularOpt(boolean linearCost, double gap)
			throws GRBException {
		System.out.println("Testing single robot, opt, on regular grids, " +
			(linearCost?"OP":"COP") + ", GAP: " + gap);
		int[] gridSizes = new int[] { 4, 5, 6};
		double maxSingleLegLength = 50;
		int budgetIncrements = 5;
		int startVertexId = 1;
		for (int s = 0; s < gridSizes.length; s++) {
			Graph g = Graph.getnxnGrid(gridSizes[s]);
			for (int l = 0; l < budgetIncrements; l++) {
				double budget = gridSizes[s] * 4 * (l + 1)
						/ (budgetIncrements * 1.0);
				Object sol[] = QCOPSolver.solveFullGraph(g, budget,
						maxSingleLegLength, linearCost, startVertexId, gap);
				if (sol[0] == null){
					System.out.println();
					resetTime();
				}
				else {
					System.out.print("Running: |V|="
							+ (gridSizes[s] * gridSizes[s]) + ", Budget="
							+ budget + ", Max leg length=" + maxSingleLegLength
							+ ", SV=" + startVertexId + ", Computation time: "
							+ elapsedTime() + " ");
					System.out.println(getOutput(sol, g));
				}
			}
		}
	}

	public static void testTwoRobotsRegularOpt(double gap) throws GRBException {
		System.out.println("Testing two-robot, on regular grids, gap: "
				+ gap);
		int[] gridSizes = new int[] { 4, 5, 6 };
		int svs[] = new int[2];
		double maxSingleLegLength = 2;
		int budgetIncrements = 5;
		for (int s = 0; s < gridSizes.length; s++) {
			Graph g = Graph.getnxnGrid(gridSizes[s]);
			svs[0] = 1;
			svs[1] = gridSizes[s] * gridSizes[s] - 2;
			for (int l = 0; l < budgetIncrements; l++) {
				double budget[] = new double[] {
						gridSizes[s] * 2 * (l + 1) / (budgetIncrements * 1.0),
						gridSizes[s] * 2 * (l + 1) / (budgetIncrements * 1.0) };
				Object sol[] = QCOPMultiSolver.solveFullGraphMulti(g, svs,
						budget, maxSingleLegLength, false, gap);
				if (sol[0] == null || ((Double) sol[0]).doubleValue() <= 0.0)
				{
					System.out.println();
					resetTime();
				}
				else {
					System.out.print("Running: |V|="
							+ (gridSizes[s] * gridSizes[s]) + ", Budget="
							+ budget[0] + ", Max leg length="
							+ maxSingleLegLength + ", Computation time: "
							+ elapsedTime() + " ");
					System.out.println(getOutput(sol, g));
				}
			}
		}
	}

	public static void testIrregularNetwork(boolean linearCost, double gap) 
			throws GRBException {
		System.out.println("Testing multiple robots on irregular network, " 
				+ (linearCost?"OP":"COP") + ", GAP: " + gap);
		int[][] svss = new int[][] { { 1 }, { 1, 14 }, { 2, 7, 16 } };
		double b[][] = new double[][]{
				{10, 15, 20, 25, 30, 35, 40}, 
				{5, 7.5, 10, 12.5, 15, 17.5, 20}, 
				{4, 5.5, 7, 8.5, 10, 11.5, 13}};
		double maxSingleLegLength = 20;
		// int budgetIncrements = 4;
		for (int s = 0; s < svss.length; s++) {
			System.out.println("" + svss[s].length + " robot(s):");
			Graph g = Graph.getIrregularTestCaseGraph();
			// g.randomize();
			for (int l = 0; l < b[s].length; l++) {

				double budget[] = new double[svss[s].length];
				for (int d = 0; d < budget.length; d++) {
					budget[d] = b[s][l]; 
				}
				Object sol[] = QCOPMultiSolver.solveFullGraphMulti(g, svss[s],
						budget, maxSingleLegLength, linearCost, gap);
				if (sol[0] == null || ((Double) sol[0]).doubleValue() <= 0.0)
				{
					System.out.println();
					resetTime();
				}
				else {
					System.out.print("Running with budget=" + budget[0]
							+ ", Max leg length=" + maxSingleLegLength
							+ ", Computation time: " + elapsedTime() + " ");
					System.out.println(getOutput(sol, g));
				}
			}

		}
	}


	public static String getOutput(Object sol[], Graph g) {
//		if (sol[0] != null) {
//			return "R: " + sol[0];
//		}

		StringBuffer sb = new StringBuffer();
		if (sol[0] != null) {
			sb.append("R:" + sol[0]);
			if (sol.length == 2) {
				sb.append("|");
			} else {
				sb.append("|");
			}
			for (int a = 0; a < sol.length - 1; a++) {
				double distance = 0;
				Vertex[] vs = ((Vector<Vertex>) (sol[a + 1]))
						.toArray(new Vertex[0]);
				for (int i = 0; i < vs.length; i++) {
					sb.append(vs[i].id + " ");
					if (i < vs.length - 1) {
						distance += g.edgeLengths[vs[i].id][vs[i + 1].id];
					}
				}
				sb.append("|" + distance + "||");
				// sb.append("\n");
			}
		}
		return sb.toString();
	}

	public static void main(String argv[]) {
		try {
			// Testing single robot, finding optimal solution on regular
			// grids
//			testSingleRobotRegularOpt(false, 0);
//			testSingleRobotRegularOpt(false, 0.02);
//			testSingleRobotRegularOpt(false, 0.10);
//			testSingleRobotRegularOpt(false, 0.25);
//			testSingleRobotRegularOpt(true, 0);

			// Testing single robot, finding sub-optimal solution on regular
			// grids
//			testSingleRobotRegularSubopt(false, 0.2);
//			testSingleRobotRegularSubopt(true, 0);
			
			// Testing single robot on irregular network
//			testIrregularNetwork(false, 0);
//			testIrregularNetwork(true, 0);
			

			// Two robots, regular grid
//			testTwoRobotsRegularSubopt();
// 			testTwoRobotsRegularOpt(0.2);
//			testThreeRobotsRegularSubopt();
			
			// Single robot, randomly perturbed grid, both at gap 0 and gap 0.2
//			testSingleRobotRandomPerformance();
			testSinglePurturbedSquare(0);
//			testSinglePurturbed(0.2);
			
			// Not used in the paper
			// testTwoRobotsRandomPerformanceSubopt();
			// testThreeRobotsRandomPerformanceSubopt();
//			testSingleRobot(false, 0);
			
		} catch (GRBException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static void testSingleRobot(boolean linearCost, double gap)
			throws GRBException {
		System.out.println("Testing single robot, opt, on regular grids, " +
			(linearCost?"OP":"COP") + ", GAP: " + gap);
		int[] gridSizes = new int[] {3, 4};
		double maxSingleLegLength = 50;
		int budgetIncrements = 5;
		int startVertexId = 1;
		for (int s = 0; s < gridSizes.length; s++) {
			Graph g = Graph.getnxnGrid(gridSizes[s]);
			for (int l = 0; l < budgetIncrements; l++) {
				double budget = l + 2 + s*8;
				Object sol[] = QCOPSolver.solveFullGraph(g, budget,
						maxSingleLegLength, linearCost, startVertexId, gap);
				if (sol[0] == null){
					System.out.println();
					resetTime();
				}
				else {
					System.out.print("Running: |V|="
							+ (gridSizes[s] * gridSizes[s]) + ", Budget="
							+ budget + ", Max leg length=" + maxSingleLegLength
							+ ", SV=" + startVertexId + ", Computation time: "
							+ elapsedTime() + " ");
					System.out.println(getOutput(sol, g));
				}
			}
		}
	}


}

