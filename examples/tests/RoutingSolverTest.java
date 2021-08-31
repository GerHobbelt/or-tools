// Copyright 2010-2021 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
package com.google.ortools;

import static java.lang.Math.abs;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.google.ortools.Loader;
import com.google.ortools.constraintsolver.Assignment;
import com.google.ortools.constraintsolver.FirstSolutionStrategy;
import com.google.ortools.constraintsolver.RoutingIndexManager;
import com.google.ortools.constraintsolver.RoutingModel;
import com.google.ortools.constraintsolver.RoutingSearchParameters;
import com.google.ortools.constraintsolver.main;
import com.google.ortools.constraintsolver.IntBoolPair;
import java.util.ArrayList;
import java.util.function.LongBinaryOperator;
import java.util.function.LongUnaryOperator;
import java.util.logging.Logger;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

/** Tests the Routing java interface. */
public final class RoutingSolverTest {
  private static final Logger logger = Logger.getLogger(RoutingSolverTest.class.getName());
  private ArrayList<Integer[]> coordinates;

  @BeforeEach
  public void setUp() {
    Loader.loadNativeLibraries();
    coordinates = new ArrayList<>();
    coordinates.add(new Integer[] {0, 0});
    coordinates.add(new Integer[] {-1, 0});
    coordinates.add(new Integer[] {-1, 2});
    coordinates.add(new Integer[] {2, 1});
    coordinates.add(new Integer[] {1, 0});
  }

  public LongBinaryOperator createManhattanCostCallback(RoutingIndexManager manager) {
    return (long i, long j) -> {
        final int firstIndex = manager.indexToNode(i);
        final int secondIndex = manager.indexToNode(j);
        final Integer[] firstCoordinate = coordinates.get(firstIndex);
        final Integer[] secondCoordinate = coordinates.get(secondIndex);
        return (long) Math.abs(firstCoordinate[0] - secondCoordinate[0])
            + Math.abs(firstCoordinate[1] - secondCoordinate[1]);
    };
  }

  public LongUnaryOperator createUnaryCostCallback(RoutingIndexManager manager) {
    return (long fromIndex) -> {
        final int fromNode = manager.indexToNode(fromIndex);
        final Integer[] firstCoordinate = coordinates.get(fromNode);
        return (long) Math.abs(firstCoordinate[0]) + Math.abs(firstCoordinate[1]);
    };
  }

  public LongBinaryOperator createReturnOneCallback() {
    return (long i, long j) -> 1;
  }

  @Test
  public void testRoutingIndexManager() {
    final RoutingIndexManager manager = new RoutingIndexManager(42, 3, 7);
    assertNotNull(manager);
    assertEquals(42, manager.getNumberOfNodes());
    assertEquals(3, manager.getNumberOfVehicles());
    assertEquals(42 + 3 * 2 - 1, manager.getNumberOfIndices());
    for (int i = 0; i < manager.getNumberOfVehicles(); ++i) {
      assertEquals(7, manager.indexToNode(manager.getStartIndex(i)));
      assertEquals(7, manager.indexToNode(manager.getEndIndex(i)));
    }
  }

  @Test
  public void testRoutingIndexManagerMultiDepotSame() {
    final RoutingIndexManager manager =
        new RoutingIndexManager(42, 3, new int[] {7, 7, 7}, new int[] {7, 7, 7});
    assertNotNull(manager);
    assertEquals(42, manager.getNumberOfNodes());
    assertEquals(3, manager.getNumberOfVehicles());
    assertEquals(42 + 3 * 2 - 1, manager.getNumberOfIndices());
    for (int i = 0; i < manager.getNumberOfVehicles(); ++i) {
      assertEquals(7, manager.indexToNode(manager.getStartIndex(i)));
      assertEquals(7, manager.indexToNode(manager.getEndIndex(i)));
    }
  }

  @Test
  public void testRoutingIndexManagerMultiDepotAllDifferent() {
    final RoutingIndexManager manager =
        new RoutingIndexManager(42, 3, new int[] {1, 2, 3}, new int[] {4, 5, 6});
    assertNotNull(manager);
    assertEquals(42, manager.getNumberOfNodes());
    assertEquals(3, manager.getNumberOfVehicles());
    assertEquals(42, manager.getNumberOfIndices());
    for (int i = 0; i < manager.getNumberOfVehicles(); ++i) {
      assertEquals(i + 1, manager.indexToNode(manager.getStartIndex(i)));
      assertEquals(i + 4, manager.indexToNode(manager.getEndIndex(i)));
    }
  }

  @Test
  public void testRoutingModel() {
    final RoutingIndexManager manager =
        new RoutingIndexManager(42, 3, new int[] {1, 2, 3}, new int[] {4, 5, 6});
    assertNotNull(manager);
    final RoutingModel model = new RoutingModel(manager);
    assertNotNull(model);
    for (int i = 0; i < manager.getNumberOfVehicles(); ++i) {
      assertEquals(i + 1, manager.indexToNode(model.start(i)));
      assertEquals(i + 4, manager.indexToNode(model.end(i)));
    }
  }

  @Test
  public void testCostsAndSolve() {
    final RoutingIndexManager manager = new RoutingIndexManager(coordinates.size(), 1, 0);
    final RoutingModel model = new RoutingModel(manager);
    assertEquals(5, model.nodes());
    final LongBinaryOperator callback = createManhattanCostCallback(manager);
    final int cost = model.registerTransitCallback(callback);
    model.setArcCostEvaluatorOfAllVehicles(cost);
    assertEquals(RoutingModel.ROUTING_NOT_SOLVED, model.status());
    final Assignment solution = model.solve(null);
    assertEquals(RoutingModel.ROUTING_SUCCESS, model.status());
    assertEquals(10, solution.objectiveValue());
  }

  @Test
  public void testRoutingTransitMatrix() {
    logger.info("testRoutingTransitMatrix...");
    // Create Routing Index Manager
    RoutingIndexManager manager =
        new RoutingIndexManager(5 /*location*/, 1 /*vehicle*/, 0 /*depot*/);
    // Create Routing Model.
    RoutingModel routing = new RoutingModel(manager);
    // Define cost of each arc.
    int transitCallbackIndex;
    long[][] matrix = {
      {1, 1, 1, 1, 1},
      {1, 1, 1, 1, 1},
      {1, 1, 1, 1, 1},
      {1, 1, 1, 1, 1},
      {1, 1, 1, 1, 1},
    };
    transitCallbackIndex = routing.registerTransitMatrix(matrix);
    routing.setArcCostEvaluatorOfAllVehicles(transitCallbackIndex);
    // Setting first solution heuristic.
    RoutingSearchParameters searchParameters =
        main.defaultRoutingSearchParameters()
            .toBuilder()
            .setFirstSolutionStrategy(FirstSolutionStrategy.Value.PATH_CHEAPEST_ARC)
            .build();
    // Solve the problem.
    Assignment solution = routing.solveWithParameters(searchParameters);
    if (null == solution)
      throw new AssertionError("null == solution");
    if (5 != solution.objectiveValue())
      throw new AssertionError("5 != objective");
    logger.info("testRoutingTransitMatrix...DONE");
  }

  @ParameterizedTest
  @ValueSource(booleans = {false, true})
  public void testRoutingTransitCallback(boolean enableGC) {
    logger.info("testRoutingTransitCallback (enable gc:" + enableGC + ")...");
    // Create Routing Index Manager
    RoutingIndexManager manager =
        new RoutingIndexManager(5 /*location*/, 1 /*vehicle*/, 0 /*depot*/);
    // Create Routing Model.
    RoutingModel routing = new RoutingModel(manager);
    // Define cost of each arc.
    int transitCallbackIndex;
    if (true) {
      transitCallbackIndex = routing.registerTransitCallback((long fromIndex, long toIndex) -> {
        // Convert from routing variable Index to user NodeIndex.
        int fromNode = manager.indexToNode(fromIndex);
        int toNode = manager.indexToNode(toIndex);
        return abs(toNode - fromNode);
      });
    }
    if (enableGC) {
      System.gc();
    }
    routing.setArcCostEvaluatorOfAllVehicles(transitCallbackIndex);
    // Setting first solution heuristic.
    RoutingSearchParameters searchParameters =
        main.defaultRoutingSearchParameters()
            .toBuilder()
            .setFirstSolutionStrategy(FirstSolutionStrategy.Value.PATH_CHEAPEST_ARC)
            .build();
    // Solve the problem.
    Assignment solution = routing.solveWithParameters(searchParameters);
    if (null == solution)
      throw new AssertionError("null == solution");
    if (8 != solution.objectiveValue())
      throw new AssertionError("8 != objective");
    logger.info("testRoutingTransitCallback (enable gc:" + enableGC + ")...DONE");
  }

  @Test
  public void testRoutingMatrixDimension() {
    logger.info("testRoutingMatrixDimension...");
    // Create Routing Index Manager
    RoutingIndexManager manager =
        new RoutingIndexManager(5 /*location*/, 1 /*vehicle*/, 0 /*depot*/);
    // Create Routing Model.
    RoutingModel routing = new RoutingModel(manager);
    // Define cost of each arc.
    int transitCallbackIndex;
    long[][] matrix = {
      {1, 1, 1, 1, 1},
      {1, 1, 1, 1, 1},
      {1, 1, 1, 1, 1},
      {1, 1, 1, 1, 1},
      {1, 1, 1, 1, 1},
    };
    IntBoolPair result = routing.addMatrixDimension(
        matrix,
        /*capacity=*/10,
        /*fix_start_cumul_to_zero=*/true,
        "Dimension");
    routing.setArcCostEvaluatorOfAllVehicles(result.getFirst());
    // Setting first solution heuristic.
    RoutingSearchParameters searchParameters =
        main.defaultRoutingSearchParameters()
            .toBuilder()
            .setFirstSolutionStrategy(FirstSolutionStrategy.Value.PATH_CHEAPEST_ARC)
            .build();
    // Solve the problem.
    Assignment solution = routing.solveWithParameters(searchParameters);
    if (null == solution)
      throw new AssertionError("null == solution");
    if (5 != solution.objectiveValue())
      throw new AssertionError("5 != objective");
    logger.info("testRoutingMatrixDimension...DONE");
  }

  @Test
  public void testRoutingUnaryTransitVector() {
    logger.info("testRoutingUnaryTransitVector...");
    // Create Routing Index Manager
    RoutingIndexManager manager =
        new RoutingIndexManager(5 /*location*/, 1 /*vehicle*/, 0 /*depot*/);
    // Create Routing Model.
    RoutingModel routing = new RoutingModel(manager);
    // Define cost of each arc.
    int transitCallbackIndex;
    long[] vector = {1, 1, 1, 1, 1};
    transitCallbackIndex = routing.registerUnaryTransitVector(vector);
    routing.setArcCostEvaluatorOfAllVehicles(transitCallbackIndex);
    // Setting first solution heuristic.
    RoutingSearchParameters searchParameters =
        main.defaultRoutingSearchParameters()
            .toBuilder()
            .setFirstSolutionStrategy(FirstSolutionStrategy.Value.PATH_CHEAPEST_ARC)
            .build();
    // Solve the problem.
    Assignment solution = routing.solveWithParameters(searchParameters);
    if (null == solution)
      throw new AssertionError("null == solution");
    if (5 != solution.objectiveValue())
      throw new AssertionError("5 != objective");
    logger.info("testRoutingUnaryTransitVector...DONE");
  }

  @ParameterizedTest
  @ValueSource(booleans = {false, true})
  public void testRoutingUnaryTransitCallback(boolean enableGC) {
    logger.info("testRoutingUnaryTransitCallback (enable gc:" + enableGC + ")...");
    // Create Routing Index Manager
    RoutingIndexManager manager =
        new RoutingIndexManager(5 /*location*/, 1 /*vehicle*/, 0 /*depot*/);
    // Create Routing Model.
    RoutingModel routing = new RoutingModel(manager);
    // Define cost of each arc.
    int transitCallbackIndex;
    if (true) {
      transitCallbackIndex = routing.registerUnaryTransitCallback((long fromIndex) -> {
        // Convert from routing variable Index to user NodeIndex.
        int fromNode = manager.indexToNode(fromIndex);
        return abs(fromNode);
      });
    }
    if (enableGC) {
      System.gc();
    }
    routing.setArcCostEvaluatorOfAllVehicles(transitCallbackIndex);
    // Setting first solution heuristic.
    RoutingSearchParameters searchParameters =
        main.defaultRoutingSearchParameters()
            .toBuilder()
            .setFirstSolutionStrategy(FirstSolutionStrategy.Value.PATH_CHEAPEST_ARC)
            .build();
    // Solve the problem.
    Assignment solution = routing.solveWithParameters(searchParameters);
    if (null == solution)
      throw new AssertionError("null == solution");
    if (10 != solution.objectiveValue())
      throw new AssertionError("10 != objective");
    logger.info("testRoutingUnaryTransitCallback (enable gc:" + enableGC + ")...DONE");
  }

  @Test
  public void testRoutingVectorDimension() {
    logger.info("testRoutingVectorDimension...");
    // Create Routing Index Manager
    RoutingIndexManager manager =
        new RoutingIndexManager(5 /*location*/, 1 /*vehicle*/, 0 /*depot*/);
    // Create Routing Model.
    RoutingModel routing = new RoutingModel(manager);
    // Define cost of each arc.
    int transitCallbackIndex;
    long[] vector = {1, 1, 1, 1, 1};
    IntBoolPair result = routing.addVectorDimension(
        vector,
        /*capacity=*/10,
        /*fix_start_cumul_to_zero=*/true,
        "Dimension");
    routing.setArcCostEvaluatorOfAllVehicles(result.getFirst());
    // Setting first solution heuristic.
    RoutingSearchParameters searchParameters =
        main.defaultRoutingSearchParameters()
            .toBuilder()
            .setFirstSolutionStrategy(FirstSolutionStrategy.Value.PATH_CHEAPEST_ARC)
            .build();
    // Solve the problem.
    Assignment solution = routing.solveWithParameters(searchParameters);
    if (null == solution)
      throw new AssertionError("null == solution");
    if (5 != solution.objectiveValue())
      throw new AssertionError("5 != objective");
    logger.info("testRoutingMatrixDimension...DONE");
  }
}
