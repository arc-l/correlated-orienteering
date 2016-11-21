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
import edu.rutgers.cs.arc.qcop.GreedyAlgorithm;
import edu.rutgers.cs.arc.qcop.QCOPSolver;
import edu.rutgers.cs.arc.qcop.Vertex;
import gurobi.GRBException;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Vector;

import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;

/**
 * 
 * @author Jingjin
 *
 *         Testing QCOP over temperature data. A greedy algorithm is also
 *         included that performs worse as budget increases.
 * 
 */
public class MATempTest {

	public static Vertex vertices[] = {
			new Vertex(0, new int[] { 1, 4 }, 42.7, -73.16667),
			new Vertex(1, new int[] { 0, 2 }, 42.42722, -73.28917),
			new Vertex(2, new int[] { 1, 3 }, 42.0931, -72.8035),
			new Vertex(3, new int[] { 2, 4, 5, 6 }, 42.3861, -72.5374),
			new Vertex(4, new int[] { 0, 3, 5 }, 42.5719, -72.5975),
			new Vertex(5, new int[] { 3, 4, 6, 7 }, 42.6617, -71.9359),
			new Vertex(6, new int[] { 3, 5, 7, 10 }, 42.2706, -71.8731),
			new Vertex(7, new int[] { 5, 6, 8, 9 }, 42.6409, -71.3637),
			new Vertex(8, new int[] { 7, 9 }, 42.8634, -70.8998),
			new Vertex(9, new int[] { 7, 8, 10, 11 }, 42.3606, -71.0106),
			new Vertex(10, new int[] { 6, 9, 11, 12 }, 41.9928, -71.1666),
			new Vertex(11, new int[] { 9, 10, 12 }, 41.90972, -70.72944),
			new Vertex(12, new int[] { 10, 11, 13 }, 41.67639, -70.95833),
			new Vertex(13, new int[] { 11, 12 }, 41.6875, -69.99333) };

	public static double data[][] = new double[14][];
	static {
		data[0] = new double[] { -2, -0.4, 6.5, 7.6, 15.9, 18.2, 21.7, 20.6,
				15.3, 11.4, 2.9, 0.9, -3.7, -3.5, 0, 7.1, 14.1, 18.3, 22.5,
				18.9, 14.9, 10.3, 2.3, -2.1 };
		data[1] = new double[] { -2.9, -1.3, 5.8, 7.6, 15.6, 17.7, 21.8, 20.2,
				14.9, 10.7, 2.2, -0.2, -4.7, -4, -0.7, 6.4, 13.4, 17.8, 22.1,
				18.5, 14.1, 9.9, 1.4, -2.7 };
		data[2] = new double[] { -1.9, 0.5, 7, 9.3, 16.6, 19.4, 24, 22.4, 17.2,
				11.4, 3.6, 0.7, -3.2, -2.1, 1.6, 8, 15.1, 20, 24.7, 20.8, 16.1,
				11.7, 3, -1.7 };
		data[3] = new double[] { -1.6, 0.8, 6.8, 8.8, 16.7, 19, 22.9, 21.9,
				16.5, 11.5, 3.6, 0.4, -3.8, -2.8, 1.5, 7.7, 14.5, 19.6, 24.2,
				19.8, 15.8, 10.8, 3, -2.3 };
		data[4] = new double[] { -2, 0.9, 6.9, 9.2, 16.5, 19, 23, 22.1, 16.9,
				11.7, 3.6, 0.7, -3.7, -2.3, 1.6, 8, 14.7, 19.4, 24, 20.1, 15.7,
				10.4, 2.5, -2.5 };
		data[5] = new double[] { -3.7, -1.3, 4.5, 7, 14.5, 17.1, 21.4, 20.8,
				15, 10.1, 2.7, -0.6, -5.5, -4.9, -1.4, 5.9, 12.6, 18, 22.2, 17,
				13.1, 9.2, 1, -3.8 };
		data[6] = new double[] { -1.6, 0.6, 6.6, 9.6, 16, 18.3, 22.9, 22.2,
				16.5, 11.7, 4, 1.2, -2.9, -2.6, 1, 7.8, 14.2, 18.8, 23.4, 19.8,
				15.8, 10.9, 3.2, -2.3 };
		data[7] = new double[] { -1.8, 0.3, 6.4, 9.8, 15.4, 18.8, 23.1, 22.5,
				17, 11.8, 3.8, 0.7, -3.2, -2.3, 1.3, 8.1, 14.3, 20, 24.3, 20.4,
				16.8, 10.7, 3.4, -3.2 };
		data[8] = new double[] { -1.2, 0.5, 6.3, 9.1, 14, 17.9, 22.7, 22.4,
				17.1, 11.9, 4.4, 1.2, -2.7, -2, 1.8, 7.3, 13, 19.8, 23.4, 20.5,
				16.7, 11, 3.9, -1.9 };
		data[9] = new double[] { 1.2, 3, 8.1, 11.7, 15.7, 19.3, 24.1, 23.7,
				18.2, 13.6, 5.6, 3.6, -0.3, -0.5, 3.2, 9.4, 14.9, 20.9, 25,
				22.3, 18.2, 13.7, 6, 0.8 };
		data[10] = new double[] { -0.2, 1.2, 6.7, 9.5, 15.7, 18.4, 23.1, 22.3,
				16.8, 11.5, 4.3, 2.5, -1.9, -1.6, 2.1, 8.4, 14.4, 20.2, 24.7,
				20.1, 16.2, 10.8, 4.5, -0.6 };
		data[11] = new double[] { 1.1, 1.9, 6.7, 10, 15.4, 18.1, 22.7, 22.7,
				16.8, 12.8, 5, 3.4, -0.8, -0.3, 2.9, 8.3, 14.6, 20, 24.7, 20.5,
				16.6, 11.7, 5.1, 0.5 };
		data[12] = new double[] { 1.5, 2.1, 7.2, 10, 15.7, 18.3, 23.1, 22.5,
				16.9, 12.9, 5.2, 3.7, -0.5, -0.3, 3.2, 8.5, 14.3, 19.8, 24.6,
				20.3, 16.4, 11.6, 5.1, 1 };
		data[13] = new double[] { 2.3, 2.6, 6, 9.6, 14.2, 17.9, 23, 22.7, 17.7,
				13.5, 6.7, 4.6, 0.3, 0.5, 3.5, 7.8, 13.6, 18.6, 23.9, 21, 17.2,
				13.3, 6.3, 2.2 };
	}

	static int sampleSize = 12;

	public static void main(String argv[]) {
		double errors[][] = new double[8][3];
		for (int i = 0; i < errors.length; i++) {
			double e[] = runBudget(2.0 + i * 1);
			errors[i][0] = 2.0 + i * 1;
			errors[i][1] = e[0];
			errors[i][2] = e[1];
		}

		for (int i = 0; i < errors.length; i++) {
			System.out.println(errors[i][0] + ", " + errors[i][1] + ", "
					+ errors[i][2]);
		}

	}

	public static double[] runBudget(double budget) {
		double errors[] = new double[2];
		// 8.3 is needed for TSP

		Graph g = new Graph();
		for (int i = 0; i < vertices.length; i++) {
			g.addVertex(vertices[i]);
		}
		g.updateNeighbors();
		g.computeEdgeLengths();

		int startVertexId = 9;

		// Do regression to populate the weights

		for (int i = 0; i < vertices.length; i++) {
			// Get all neighbors
			int nbrsIds[] = vertices[i].nbrsIds;
			vertices[i].beta = new double[nbrsIds.length + 1];

			// Prepare y samples
			double samplesY[] = new double[sampleSize];
			for (int d = 0; d < sampleSize; d++) {
				samplesY[d] = data[i][d];
			}

			// Prepare x samples (single variable)
			for (int v = 0; v < nbrsIds.length; v++) {
				double samplesX[][] = new double[sampleSize][1];
				// For X
				for (int d = 0; d < sampleSize; d++) {
					samplesX[d][0] = data[nbrsIds[v]][d];
				}

				// Run regression
				OLSMultipleLinearRegression regression = 
						new OLSMultipleLinearRegression();
				regression.newSampleData(samplesY, samplesX);
				vertices[i].beta[v + 1] = regression
						.estimateRegressionParameters()[1];
			}

			// Build weights. First sum up all coeffs
			double sumCoeff = 0;
			for (int n = 0; n < nbrsIds.length; n++) {
				sumCoeff += (Math.abs(vertices[i].beta[n + 1]));
			}

			for (int n = 0; n < nbrsIds.length; n++) {
				vertices[i].weights[n] = (Math.abs(vertices[i].beta[n + 1]))
						/ sumCoeff;
			}

			// Prepare samples for all neighbors
			double samplesX[][] = new double[sampleSize][nbrsIds.length];

			// For y
			for (int d = 0; d < sampleSize; d++) {
				samplesY[d] = data[i][d];
			}
			// For X
			for (int n = 0; n < nbrsIds.length; n++) {
				for (int d = 0; d < sampleSize; d++) {
					samplesX[d][n] = data[nbrsIds[n]][d];
				}
			}

			// Run regression
			OLSMultipleLinearRegression regression = 
					new OLSMultipleLinearRegression();
			regression.newSampleData(samplesY, samplesX);
			double beta[] = new double[nbrsIds.length + 1];
			beta = regression.estimateRegressionParameters();
			for (int n = 0; n < beta.length; n++) {
				vertices[i].beta[n] += beta[n];
			}

			sumCoeff = 0;
			// Build weights. First sum up all coeffs
			for (int n = 0; n < nbrsIds.length; n++) {
				sumCoeff += Math.abs(vertices[i].beta[n + 1]);
			}
			for (int n = 0; n < nbrsIds.length; n++) {
				vertices[i].weights[n] = Math.abs(vertices[i].beta[n + 1])
						/ sumCoeff;
				vertices[i].weights[n] *= 0.7;
			}
		}

		// Update graph
		g.computeEdgeLengths();

		// g.print();
		Vertex[] solution = new Vertex[0];

		try {
			Object[] ret = QCOPSolver.solveFullGraph(g, budget, 5, false,
					startVertexId, 0);

			// System.out.print("Optimal raw value: " + ret[0] + "  ");
			Vector<Vertex> vVec = (Vector<Vertex>) ret[1];
			solution = vVec.toArray(new Vertex[0]);
			System.out.println(((Double) ret[0]).doubleValue());
			for (int i = 0; i < solution.length; i++) {
				System.out.print(solution[i].id
						+ (i != solution.length - 1 ? "-" : " "));
			}
			// System.out.println();
			// double totalError = estimateError(vertices, solution);
			errors[0] = estimateValueRecursive(vertices, solution);
			// System.out.print("Total error: " + totalError);

		} catch (GRBException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// Compute a heuristic solution
		{
			Vertex[] hSolution = new Vertex[0];
			hSolution = GreedyAlgorithm.solve(g, startVertexId, budget)
					.toArray(new Vertex[0]);
			// double totalError = estimateError(vertices, hSolution);
			// System.out.println("Total error: " + totalError);
			errors[1] = estimateValueRecursive(vertices, hSolution);
		}

		return errors;

	}

	/**
	 * Estimat error from a given set of visited vertices
	 * 
	 * @param vs
	 * @param n
	 * @param visited
	 * @return
	 */
	public static double estimateError(Vertex[] vs, Vertex[] visited) {
		double totalError = 0;
		// Put all solution vertices into a map for easy lookup
		Map<Integer, Vertex> visitedVertices = new HashMap<Integer, Vertex>();
		for (Vertex v : visited) {
			visitedVertices.put(v.id, v);
		}

		System.out.println();

		for (int s = 0; s < 4; s++) {
			int step = sampleSize + 3 * s - 1;
			for (int i = 0; i < vs.length; i++) {
				// Skip if the vertex is visited
				if (!visitedVertices.containsKey(vs[i].id)) {
					// Now get neighbors of the vertex in the set of visited
					// vertices
					Vector<Vertex> availableNbrs = new Vector<Vertex>();
					for (Vertex v : vs[i].neighbors) {
						if (visitedVertices.containsKey(v.id)) {
							availableNbrs.add(v);
						}
					}

					if (availableNbrs.size() == 0) {
						// If there is no neighbor, error is 1
						totalError += 1;
					} else {
						// Otherwise, get data to do estimate
						double[] y = new double[sampleSize];
						for (int d = 0; d < sampleSize; d++) {
							y[d] = data[vs[i].id][d];
						}

						double[][] X = new double[sampleSize][availableNbrs
								.size()];
						for (int j = 0; j < availableNbrs.size(); j++) {
							Vertex v = availableNbrs.get(j);
							for (int d = 0; d < sampleSize; d++) {
								X[d][j] = data[v.id][d];
							}
						}

						// Do regression
						OLSMultipleLinearRegression regression = 
								new OLSMultipleLinearRegression();
						regression.newSampleData(y, X);
						double[] beta = regression
								.estimateRegressionParameters();

						// Make estimate at a few steps
						// double error = 0;
						double value = beta[0];
						for (int ix = 1; ix < beta.length; ix++) {
							value += data[availableNbrs.get(ix - 1).id][step]
									* beta[ix];
						}
						System.out.print(value); // + " " + data[vs[i].id][step]
						// error += Math.abs((value -
						// data[vs[i].id][step])/(data[vs[i].id][step]));
						totalError += Math.abs(value - data[vs[i].id][step]);
					}
				}

				else {
					System.out.print(data[vs[i].id][step] + " ");
				}

				System.out.print(",");
			}
			System.out.println();
		}
		return totalError;
	}

	public static double estimateValueRecursive(Vertex[] vs, Vertex[] visited) {
		System.out.println();

		double totalError = 0;
		for (int s = 0; s < 4; s++) {
			int step = sampleSize + 3 * s - 1;
			// Put all solution vertices into a map for easy lookup
			HashSet<Integer> visitedVertices = new HashSet<Integer>();
			for (Vertex v : visited) {
				visitedVertices.add(v.id);
				v.importance = data[v.id][step];
			}

			while (visitedVertices.size() != vs.length) {
				HashSet<Integer> tempVisitedVertices = new HashSet<Integer>();
				for (int i = 0; i < vs.length; i++) {
					// Skip if the vertex is visited
					if (!visitedVertices.contains(vs[i].id)) {
						// Now get neighbors of the vertex in the set of visited
						// vertices
						Vector<Vertex> availableNbrs = new Vector<Vertex>();
						for (Vertex v : vs[i].neighbors) {
							if (visitedVertices.contains(v.id)) {
								availableNbrs.add(v);
							}
						}

						if (availableNbrs.size() == 0) {
							continue;
						} else {
							// Otherwise, get data to do estimate
							double[] y = new double[sampleSize];
							for (int d = 0; d < sampleSize; d++) {
								y[d] = data[vs[i].id][d];
							}

							double[][] X = new double[sampleSize][availableNbrs
									.size()];
							for (int j = 0; j < availableNbrs.size(); j++) {
								Vertex v = availableNbrs.get(j);
								for (int d = 0; d < sampleSize; d++) {
									X[d][j] = data[v.id][d];
								}
							}

							// Do regression
							OLSMultipleLinearRegression regression = 
									new OLSMultipleLinearRegression();
							regression.newSampleData(y, X);
							double[] beta = regression
									.estimateRegressionParameters();

							// Make estimate at a few steps
							double value = beta[0];
							for (int ix = 1; ix < beta.length; ix++) {
								value += availableNbrs.get(ix - 1).importance
										* beta[ix];
							}
							vs[i].importance = value;
							tempVisitedVertices.add(vs[i].id);
						}
					}
				}
				visitedVertices.addAll(tempVisitedVertices);
			}

			for (int i = 0; i < vs.length; i++) {
				System.out.print(vs[i].importance + ",");
				totalError += Math.abs(data[vs[i].id][step] - vs[i].importance);
			}
			System.out.println();
		}
		System.out.println("Total error: " + totalError);
		return totalError;
	}

}
