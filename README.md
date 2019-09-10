**University of Pennsylvania, CIS 565: GPU Programming and Architecture,
Boids Flocking**

* Grace Gilbert
  * gracelgilbert.com
* Tested on: Windows 10, i9-9900K @ 3.60GHz 64GB, GeForce RTX 2080 40860MB

![](/images/CoverGif.gif)

## Overview
This project implements a flocking simulation using the Reynolds Boids algorithm.  A flocking simulation is an artificial life model, where particles (boids) behave and interact with each other in a natural way.  The boids travel around the simulation space according to three rules:
### Rule 1 - Cohesion 
This rule causes boids to move towards other boids within their nearby neighborhood.  Boids move towards the center of mass of their neighboring boids within a certain radius.

Pseudocode for cohesion:
```
function rule1(Boid boid)

    Vector perceived_center

    foreach Boid b:
        if b != boid and distance(b, boid) < rule1Distance then
            perceived_center += b.position
        endif
    end

    perceived_center /= number_of_neighbors

    return (perceived_center - boid.position) * rule1Scale
end
```

Below is the simulation with only cohesion activated.  Notice how the boids clump together in small neighborhoods.
![](/images/cohesionOnly.gif)

### Rule 2 - Separation
The separation rule allows boids to stay some distance away from other objects, in this case other boids.  For neighboring objects within a certain radius, a boid will adjust its velocity to move away from that object and avoid it.  This rule prevents the boids from continuously colliding into each other.

Pseudocode for separation:
```
function rule2(Boid boid)

    Vector c = 0

    foreach Boid b
        if b != boid and distance(b, boid) < rule2Distance then
            c -= (b.position - boid.position)
        endif
    end

    return c * rule2Scale
end
```
### Rule 3 - Alignment
Boids try to match the direction and speed of their neighbors. Boids look at their neighbors within a certain radius and take on an average of their velocities, enabling the boids to move together with similar velocities.  
Pseudocode for alignment:
```
function rule3(Boid boid)

    Vector perceived_velocity

    foreach Boid b
        if b != boid and distance(b, boid) < rule3Distance then
            perceived_velocity += b.velocity
        endif
    end

    perceived_velocity /= number_of_neighbors

    return perceived_velocity * rule3Scale
end
```
### Rule integration
Each of the three rules produce a single change in velocity value for each boid.  To obtain the boid's final change in velocity value, we scale each of the three velocities and sum them together, adding them to the boid's previous frame velocity.  The scale values for each rule, as well as the neighborhood radii within each rule, alter the way the boids move with each other.

## Naive Approach
The first implementation uses a naive, brute force approach.  This is the least efficient of the three implementations.  For each boid, we iterate over all other boids in the simulation and check if they are within the radius of each rule and finally apply the rules. This is very inefficient, as many of the boids will not be near the neighborhood of the current boid, and we are still iterating through all of them.  A more efficient approach would be to avoid having to iterate over boids that are absolutely not going to be close enough to affect the current boid. 

## Uniform Grid
In this approach, we divide the simulation space into a uniform grid.  Each boid's position can now be found in a single cell in the grid.  The ID of the grid cell is determined by transforming the position to a space where each grid is dimension 1 oriented at the origin, and then flooring this position value to get an integral 3 dimensional index of the grid cell.  We can then use this grid to help narrow down the neighborhood search space.
### Scattered
The first uniform grid implementation is with a scattered grid. In this implementation, when we are determining the change in velocity of a boid, we limit the search area to a box of grid cells surrounding the current boid's cells.  The bounds of this box depends on the radii for each of the rules. To ensure that we are covering all potential cells with neighbors, we first find the maximum rule radius, so the maximum of the cohesion radius, separation radius, and alignment radius.  For the lower bound of the search box, we subtract this max radius from each dimension of the boid's position, convert it to grid cell space, and floor the value to include the lower bound grid cells. For the upper bound of the search box, we add the max radius, converto cell space, and then take the ceiling of it to ensure that we are getting the upper bound and are not rounding down and missing potential neighbors.  

This system ensures that we are getting the best size search box.  If instead we just had a hard coded search box size, we may be overestimating and searching through more grid cells than necessary, or vice versa nad end up missing grid cells.  

To achieve this method, we start by computing the grid index for each boid, creating a map from boid index to grid cell index.  We then sort this map according to grid cell index, enabling us to iterate through the grid cell indices and determine which boids belong in each cell.  When we first hit a new grid cell index, we know we have entered a new cell, and all corresponding boid indices belong to that grid cell, until a new cell index is found.  Therefore, once we determine which grid cells are in our search block, we can find the boid indices to iterate over.  We then have a separate map of boid indices to their positions and velocities, which are needed to apply the behavioral rules.

The diagrams below illustrate this flow of data:

![buffers for generating a uniform grid using index sort](images/Boids%20Ugrids%20buffers%20naive.png)

### Coherent
The second uniform grid implementation shares the same search algorithm as the scattered grid, but the organization of the index data is altered to be more efficient.  In the previous approach, once we figured out which grid cells to search through, we found the boids in those cells, but then had to access another buffer, particlArrayIndices, to determine the index to use to access the position and velocity data.  This approach cuts out this middle step of accessing particleArrayIndices.  To do this, we reshuffle the position and velocity data to match the way the boid indices were reshuffled when we sorted by grid cell index.  This way, we now have a direct mapping from grid cell start and end values to positions and velocities.  
The diagrams below illustrate this flow of data for the coherent method:

![buffers for generating a uniform grid using index sort, then making the boid data coherent](images/Boids%20Ugrids%20buffers%20data%20coherent.png)

## Performance Analysis

## Questions



