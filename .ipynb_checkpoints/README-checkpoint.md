
[image1]: ./images/p100_2.png
[image2]: ./images/p2_2.png


## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data. 

Your job is to build out the methods in `particle_filter.cpp` until the simulator output says:

```
Success! Your particle filter passed!
```

# Implementing the Particle Filter

The functions of the particle filter can be found in 'particle_fileter.cpp'.

`init()` function takes the GPS position to initialize the particles.

`prediction()` function calculates the state of each particle using given velocity and yar_rate data.

`dataAssociation()` function associates the observations with the nearest landmark.

`updateWeights()` function calculates the weights of each particle based on the observations.

`resample()` function selects particles according to their weights.

 

#### The Map
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.



## Success Criteria
If your particle filter passes the current grading code in the simulator then you should pass! 

The things the grading code is looking for are:


1. **Accuracy**: your particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: your particle filter should complete execution within the time of 100 seconds.


## Running


First is the running result with 100 particles:



![image1]

The average error of 100 particles is:
```
x   :   0.115

y   :   0.107

yaw :   0.004
```

Then I tried with less partiles.
with 1 particle, it failed.




But with 2 particles, I rebuild and run the prgram for 10 times, it always success, alrough the error is much larger.

![image2]

The average error of 2 particles is:
```
x   :   0.444

y   :   0.405

yaw :   0.014
```

## Reflection

The problem is simplified and idealied, which I can understand, but pass with 2 particles? Perhaps it is the problem with my program?



