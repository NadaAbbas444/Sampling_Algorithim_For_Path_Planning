## Fast Marching Trees(FMT)

Parameters:

- Research radius : The formula mathsqrt((math.log(n)˙ / n)) is a way to achieve this shrinking radius, where the math.log(n) / n term goes to zero as n (nu.of.samples) grows large, effectively reducing the radius.
- Number of samples

![tree_growth_FMT_map0](GIF/tree_growth_FMT_map0.gif)
![tree_growth_FMT_map1](GIF/tree_growth_FMT_map1.gif)
![tree_growth_FMT_map2](GIF/tree_growth_FMT_map2.gif)

## Batch Informed Trees(BIT)

The algorithm can be summarized as follows:

- Divide the search space into a number of overlapping regions.
- Initialize a tree in each region with the start node.
- Expand all trees in parallel until the goal is reached.
- Find the cheapest path to the goal from all trees and connect them. Repeat steps 3 and 4 until convergence.

![BIT_map0](GIF/BIT_map0.gif)
![BIT_map1](GIF/BIT_map1.gif)
![BIT_map2](GIF/BIT_map2.gif)

## Comparative Analysis (FMT,BIT,Informed RRTstar)

The three motion planning algorithms, Informed RRT*, Fast Marching Trees (FMT), and Batch Informed Trees (BIT), have different characteristics regarding optimality and speed. Informed RRT* aims for global optimality, while FMT prioritizes speed and efficiency. BIT strikes a balance between optimality and speed by combining aspects of Informed RRT* and FMT.

Informed RRT* is known for its extreme optimality, FMT excels in speed, and BIT offers a compromise between the two. A visual representation, such as a figure illustrating speed and cost metrics on the same map, can demonstrate how Informed RRT* achieves high optimality, FMT prioritizes speed, and BIT provides a desirable compromise for motion planning tasks.

![Combined Image Map 1](Analysis_results/combined_image_map1.jpg)
![Combined Image Map 2](Analysis_results/combined_image_map2.jpg)
![Combined Image Map 3](Analysis_results/combined_image_map3.jpg)
