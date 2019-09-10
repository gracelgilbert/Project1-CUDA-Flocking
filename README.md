**University of Pennsylvania, CIS 565: GPU Programming and Architecture,
Boids Flocking**

* Grace Gilbert
  * gracelgilbert.com
* Tested on: Windows 10, i9-9900K @ 3.60GHz 64GB, GeForce RTX 2080 40860MB

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

## Uniform Grid
### Scattered
### Coherent

## Performance Analysis





