package kyberlib.auto.pathing


import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Translation2d
import kyberlib.odd
import java.util.*
import kotlin.collections.ArrayList


// https://github.com/ReadyPlayer2/TSP
typealias Waypoint = Translation2d
typealias Route = ArrayList<Waypoint>
// todo: add more algorithms just for fun

/**
 * Implementation of various Traveling Saleman Problem Solution.
 * This is about finding the optimal path that goes to a list of points
 *
 * Sources:
 * - https://web.stanford.edu/class/cme334/docs/2012-11-14-Firouz_TSP.pdf
 * - https://github.com/ReadyPlayer2/TSP
 */
class TravelingSalesman(
    private val waypoints: MutableList<Translation2d>,
    private val startPosition: Waypoint? = null, private val endPosition: Waypoint? = null)
{
    constructor(vararg waypoints: Translation2d, startPosition: Waypoint? = null, endPosition: Waypoint? = null) : this(waypoints.toMutableList(), startPosition, endPosition)
    /**
     * Calculate the shortest route by trying every combination O(n!)
     */
    fun bruteForce(): Route {
        // Calculate
        permute(isBruteForce =  true)
        return findShortestPermutation(permutations)
    }
    /**
     * Calculates shortest route using nearest neighbour algorithm
     */
    fun nearestNeighbour(): Route {
        // Note: doesn't work with required end point
        // New route with start as Stoke
        val availableCities = waypoints.toMutableList()
        val route = Route()
        val first = startPosition ?: availableCities.removeFirst()
        while (route.size < this.waypoints.size) {
            val currentCity = route.last()
            val nearestCity = availableCities.minByOrNull { it.getDistance(currentCity) }
            if (nearestCity != null) {
                // Update current location, add to route, set current as visited
                route.add(nearestCity)
                availableCities.remove(nearestCity)
            } else break
        }
        if (endPosition != null) route.add(endPosition)
        return route
    }
    /**
     * Calculates the shortest route using branch and bound algorithm
     */
    fun branchAndBound(): Route {
        // Calculate
        permute()
        return findShortestPermutation(permutations)
    }

    /**
     * Calculates the optimal path uses recursive permutation. Faster solve but uses storage.
     */
    fun dynamic(): Route {
        // https://www.geeksforgeeks.org/travelling-salesman-problem-set-1/
        // O(n^2 * 2^n)
        return Route(dynamicRecursive(waypoints.toTypedArray()).first.toMutableList())
    }
    private fun dynamicRecursive(points: Array<Waypoint?>): Pair<Array<Waypoint?>, Double> {
        if (points.size == 2) return Pair(points, points.first()!!.getDistance(points.last()))
        var bestRoute = arrayOfNulls<Waypoint>(points.size-1);
        var shortestPath = Double.MAX_VALUE
        points.forEach {
            val pointsCopy = points.toMutableList()
            pointsCopy.remove(it)
            val subRecursion = dynamicRecursive(pointsCopy.toTypedArray())
            val path = subRecursion.first
            val pathLength = subRecursion.second
            if (pathLength < shortestPath) {
                bestRoute = path
                shortestPath = pathLength
            }

        }
        return Pair(bestRoute, shortestPath)
    }

    fun simulatedAnnealing() {}
    fun localSearch() {}
    fun genetic() {}
    fun antColony() {
        // https://en.wikipedia.org/wiki/Ant_colony_optimization_algorithms
    }

    fun christofides(): Route {
        // https://en.wikipedia.org/wiki/Christofides_algorithm

        // 1) create Minimal Spanning Tree
        val MST = prim(waypoints)

        // get the odd nodes
        val odds = MST.vertices.filter { it.nodeLengthFromRoot.odd }

        // Form the subgraph of G using only the vertices of O
        val subgraph = Tree().apply {
            vertices.addAll(odds)
            dense()
        }

        // todo: continue
        // 2) Construct a minimum-weight perfect matching M in this subgraph
        // 3) Unite matching and spanning tree T ∪ M to form an Eulerian multigraph
        // 4) Calculate Euler tour - https://www.geeksforgeeks.org/euler-tour-tree/
        val tour = Route()

        // remove repeated visits
        return Route(tour.toSet())
    }

    /**
     * Creates a minimum spanning tree using Prim's algorithm
     */
    private fun prim(verts: MutableList<Waypoint>): Tree {
        // https://www.geeksforgeeks.org/prims-minimum-spanning-tree-mst-greedy-algo-5/
        val nodeCount = verts.size
        val nodes = verts.map { Node(it) }
        val startingNode: Node = nodes.first()

        val unvisitedNodes = nodes.toMutableList()
        val visitedNodes = mutableListOf(startingNode)
        unvisitedNodes.remove(startingNode)

        val graph = Tree()
        while (visitedNodes.size != nodeCount) {
//             we mask non-exist edges with -- so it doesn't crash the argmin
            var bestPair = Pair(visitedNodes.first(), unvisitedNodes.first())
            var shortestDistance = Double.POSITIVE_INFINITY
            visitedNodes.forEach { start ->
                unvisitedNodes.forEach { end ->
                    val distance = start.position.getDistance(end.position)
                    if (distance < shortestDistance) {
                        shortestDistance = distance
                        bestPair = Pair(start, end)
                    }
                }
            }
            val newNode = Node(bestPair.second.position, bestPair.first)
            graph.addNode(newNode)
            unvisitedNodes.remove(bestPair.second)
            visitedNodes.add(newNode)
        }
        return graph
    }

    private var permutationMin = Double.MAX_VALUE
    private val permutations = ArrayList<Route>()
    /**
     * Generates all permutations in lexicographic order
     *
     * @param r: optional starting route, defaults to empty
     * @param isBruteForce: whether to try all options or only better options, defaults to false
     */
    private fun permute(r: Route = if (startPosition != null) Route(listOf(startPosition)) else Route(),
                        isBruteForce: Boolean = false) {
        val maxSize = if (startPosition == null) waypoints.size else waypoints.size + 1
//        println("Max: $maxSize, size: ${r.size}")
        if (r.size != maxSize) {
            for (city in waypoints) {
                if (r.contains(city)) continue
                // copy
                val newRoute = r.toMutableList() as Route

                // Add the first city from notVisited to the route
                newRoute.add(city)
                if (isBruteForce) {
                    // Recursive call
                    permute(newRoute, isBruteForce)
                }
                else {
                    // If a complete route has not yet been created keep permuting
                    if (permutations.isEmpty()) {
                        // Recursive call
                        permute(newRoute, isBruteForce)
                    }
                    else if (cost(newRoute) < permutationMin) {
                        // Current route cost is less than the best so far so keep permuting
                        permute(newRoute, isBruteForce)
                    }
                }
            }
        }
        else {
            if (endPosition != null) r.add(endPosition)
            permutations.add(r)
            if (!isBruteForce && cost(r) < permutationMin) {
                permutationMin = cost(r)
            }
        }
    }

    /**
     * find the shortest route in the list
     *
     * @param routeList: list of routes to Search
     */
    private fun findShortestPermutation(routeList: Collection<Route>): Route {
        // Loop through all the permutations
        return routeList.minByOrNull { cost(it) }!!
    }

    /**
     * Find the length of the route
     */
    private fun cost(route: Route): Double {
        var sum = 0.0
        for (i in 0 until route.size-1) {
            sum += route[i].getDistance(route[i+1])
        }
        return sum
    }
}

internal class MinimalSpanningTree(verts: ArrayList<Waypoint>) {
}



/**
 * Class to test various parts of the TravellingSalesman class
 */
internal object TravellingSalesmanTest {
    /**
     * Entry point
     */
    @JvmStatic
    fun main(args: Array<String>) {
        // Used to determine number of times the three algorithms should run
        val numIterations = 1
        val alg = TravelingSalesman(
            Translation2d(1.0, 1.0),
            Translation2d(4.0, 1.0),
            Translation2d(2.0, 4.0),
            Translation2d(1.0, 5.0),
            Translation2d(1.0, 10.0)
        )

        // Only individual algorithms should be run during profiling
        val timer = Timer()
        for (i in 0 until numIterations) {
            timer.start()
            // Run brute force
            alg.bruteForce()
            println("Brute Force: ${timer.get()} ms")
            timer.reset()
            // Run nearest neighbour
            alg.nearestNeighbour()
            println("NN: ${timer.get()} ms")
            timer.reset()
            // Run branch and bound
            alg.branchAndBound()
            println("B&B: ${timer.get()} ms")
            timer.stop()
        }
        // Output rough memory usage (profiler is more accurate)
        println(
            "KB: " + (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()).toDouble() / 1024
        )
    }
}