## Fast Marching Trees(FMT)

Parameters

- Research radius : The formula mathsqrt((math.log(n)˙ / n)) is a way to achieve this shrinking radius, where the math.log(n) / n term goes to zero as n (nu.of.samples) grows large, effectively reducing the radius.
- Number of samples
2  Algorithm Breakdown
- Step (1):Initialization - Initializes the algorithm and sets up the starting point, z, as the initial point. It also calculates the radius within which the algorithm will look for nearby nodes,

and initializes an empty list to keep track of the visited nodes.

- In each iteration of the loop, the algorithm identifies a set of nearby points, X~~ near, and adds the current point, z, to the visited list.
- Step (2): Finding minimum cost path: - finds the minimum-cost path from the current point, z, to each nearby point in X~~ near. It does this by finding the nearest point in the V~~ open set and calculating the cost of moving from that point to x, and then selecting the minimum cost path. If the path does not collide with obstacles, the code adds x to the V~~ open~~ new set, removes it from the V~~ unvisited set, updates its cost and parent nodes, and adds the edge connecting x

to its parent to the visited edges list.

- updates the open and closed sets, removes the current point, z, from the V~~ open set, and adds it to the V~~ closed set. adds the current point to the visited nodes set. If the V~~ open set becomes empty, the algorithm terminates. Otherwise, it selects the point in V~~ open with the lowest cost as the new current point, z.

![](Aspose.Words.ac89bb12-6d92-4b37-a9b3-65e497ce4dc5.001.png)

1

![](Aspose.Words.ac89bb12-6d92-4b37-a9b3-65e497ce4dc5.002.png)

![](Aspose.Words.ac89bb12-6d92-4b37-a9b3-65e497ce4dc5.003.png)

Figure 1: FMT results2on map0 and map1
