# Boids Flocking Algorithm for Sanitation and Hygiene Operations

## Project Overview

This project implements the Boids Flocking Algorithm, initially conceptualized by Craig Reynolds, to enhance sanitation and hygiene operations. By leveraging the principles of alignment, cohesion, and separation, we aim to create a swarm of robots capable of efficiently covering large areas for tasks such as autonomous navigation, surveillance, monitoring, and search and rescue operations. The primary goal is to apply this algorithm within the health sector, particularly for clearing areas of infectants, ensuring meticulous attention to detail and comprehensive coverage.

## Objectives

- Implement the Boids algorithm for robotic swarms.
- Study and analyze flocking behavior for sanitation applications.
- Optimize robot trajectories for complete area coverage.
- Enhance safety and cleanliness in target environments.

## Mathematical Model

### Approach 1: Boids Algorithm
Implemented by Sai Manikanta Badiga and Jonathan Ranjith Thomas

- Boid agents \( B = \{b_i, i = 1, 2, 3, \ldots, n\} \).
- Boids within the field of view \( N_i = \{b_i \in B; \forall b_j: |b_j - b_i| \leq L, j = 1, 2, 3, \ldots, f_a\} \).
- Position \( l_i \) and velocity \( v_i \) of Boid \( b_i \).
- Alignment \( a_i \), cohesion \( c_i \), and separation \( s_i \) terms.
- Next position \( l_j = l_i + v_i + X_a a_i + X_c c_i + X_s s_i \).

### Approach 2: Potential Fields
Implemented by Yash Palliwal and Venkata Sai Deepak Mutta

- System of \( N \) robots with position \( r_i \), velocity \( v_i \), and acceleration \( a_i \).
- Heading \( \theta_i = \arctan2(\dot{r_i}, \dot{r_i}) \).
- Control input \( a_i = a_{r_i} + a_{\theta_i} \).
- Stability and convergence analysis using potential functions.

## Implementation

The algorithms were implemented using the Robotarium package in Python. The simulations were performed for both approaches, with additional libraries like matplotlib used for data plotting and analysis.

### Approach 1: Results
- Random initialization and obstacle avoidance behavior.
- Coordinated flock movement and velocity convergence.
- Graphs showing positions and velocities along x and y axes.

### Approach 2: Results
- Random initialization and coordinated flock movement.
- Convergence of distances between neighboring robots and net total velocities.
- Stability of separation distance as shown in simulations.

## Videos of Implementation

- [Approach 1 Video](https://drive.google.com/file/d/1Bz3fsWjCbDzAS1rCxRTkgZRkG36jTzro/view?usp=sharing)
- [Approach 2 Video](https://drive.google.com/file/d/1QzzhDfIMTStB-udIou6gR6RJY4wsfC_w/view?usp=sharing)

## References

1. [Boids Algorithm by Van Hunter Adams](https://vanhunteradams.com/Pico/Animal_Movement/Boids-algorithm.html)
2. Choi T, Ahn C. "Artificial life based on boids model and evolutionary chaotic neural networks for creating artworks." Swarm Evol Comput 2019;47:80–9.
3. Tanner, H. G., Jadbabaie, A., & Pappas, G. J. (2003). "Stability of Flocking Motion." University of Pennsylvania. Technical Report No: MS-CIS-03-03.
4. R. Olfati-Saber, "Flocking for multi-agent dynamic systems: algorithms and theory," IEEE Transactions on Automatic Control, vol. 51, no. 3, pp. 401-420, March 2006, doi: 10.1109/TAC.2005.864190.
