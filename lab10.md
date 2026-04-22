---
layout: default
---

# Lab 10 Overview:
In this lab, I used a Python simulation to localize a virtual robot in a 2D environment similar to the environment set up in our lab space, implementing a full Bayes filter from the given setup/instruction.

(No extensions used this lab)

````Final Wordcount: 560```

## Localization and Bayes Filtering
Robots, like our cars, need to infer position from sensors and control inputs due to their lack of knowledge of the environment. A Bayes filter helps perform this probabilistically by maintaining a belief over every possible pose and updating that belief as new information arrives. The state is `(x, y, θ)`, and the arena is discretized into a given `12x9x18` grid where each cell covers 1ft x 1ft x 20°.

The filter alternates two steps. The prediction step propagates belief forward using the odometry motion model (increasing uncertainty because odometry is inherently noisy). The update step then multiplies that prediction by the sensor likelihood from 18 ToF beams spaced every 20°, which brings the belief back down to something more tangible. 

## Task: Implementing the Filter
I wrote the five required functions in the provided notebook. Conceptually they map directly onto the lecture pseudocode.

#### Compute Control
`compute_control()` decomposes the difference between two poses into the odometry model's three parameters: an initial rotation to face the next position, a translation along that heading, and a final rotation into the new orientation. `arctan2` handles the heading, `hypot` handles the distance, and both rotations get wrapped to `(-180°, 180°)` with `mapper.normalize_angle()` so the filter doesn't blow up near the wrap boundary as shown.

```python
def compute_control(cur_pose, prev_pose):
    cur_x, cur_y, cur_theta = cur_pose
    prev_x, prev_y, prev_theta = prev_pose
    rot_1 = mapper.normalize_angle(np.degrees(np.arctan2(cur_y - prev_y, cur_x - prev_x)) - prev_theta)
    trans = np.hypot(cur_y - prev_y, cur_x - prev_x)
    rot_2 = mapper.normalize_angle(cur_theta - prev_theta - rot_1)
    return rot_1, trans, rot_2
```

#### Odometry Motion Model
`odom_motion_model()` returns `p(x'|x, u)` by evaluating each of the three odometry parameters against the commanded `u` as a Gaussian with the provided rotation and translation sigmas. The three terms are multiplied under an independence assumption. This is the workhorse the prediction step calls inside its inner loop (Shown below).

```python
def odom_motion_model(cur_pose, prev_pose, u):
    rot_1, trans, rot_2 = compute_control(cur_pose, prev_pose)
    prob_rot_1 = loc.gaussian(rot_1, u[0], loc.odom_rot_sigma)
    prob_trans = loc.gaussian(trans,  u[1], loc.odom_trans_sigma)
    prob_rot_2 = loc.gaussian(rot_2, u[2], loc.odom_rot_sigma)
    return prob_rot_1 * prob_trans * prob_rot_2
```

#### Prediction Step
```prediction_step()``` uses the previous and current odometry states & odometry model to itererate over every `(previous, current)` state pair. These probabilities are multiplied and accumulated in `bel_bar` and normalized to sum to 1. I used a `0.001` cutoff on the previous belief to skip cells that contribute essentially nothing, which brought iteration time down to something tolerable as shown.

```python
def prediction_step(cur_odom, prev_odom):
    u = compute_control(cur_odom, prev_odom)
    loc.bel_bar = np.zeros((mapper.MAX_CELLS_X, mapper.MAX_CELLS_Y, mapper.MAX_CELLS_A))
    for prev_x in range(mapper.MAX_CELLS_X):
        for prev_y in range(mapper.MAX_CELLS_Y):
            for prev_theta in range(mapper.MAX_CELLS_A):
                if (loc.bel[prev_x, prev_y, prev_theta] < 0.001): continue
                for cur_x in range(mapper.MAX_CELLS_X):
                    for cur_y in range(mapper.MAX_CELLS_Y):
                        for cur_theta in range(mapper.MAX_CELLS_A):
                            p = odom_motion_model(mapper.from_map(cur_x, cur_y, cur_theta),
                                                  mapper.from_map(prev_x, prev_y, prev_theta), u)
                            loc.bel_bar[cur_x, cur_y, cur_theta] += p * loc.bel[prev_x, prev_y, prev_theta]
    loc.bel_bar = loc.bel_bar / np.sum(loc.bel_bar)
```

#### Sensor Model
`sensor_model()` returns the per-reading likelihood array for a candidate pose by evaluating each ToF reading as a Gaussian centered on the expected range with `sensor_sigma`. The 18 values get combined downstream in the update step (Shown below).

```python
def sensor_model(obs):
    return [loc.gaussian(obs[i], loc.obs_range_data[i], loc.sensor_sigma)
            for i in range(mapper.OBS_PER_CELL)]
```

#### Update Step
Finally `update_step()` scales `bel_bar` at each cell by the product of its 18 reading likelihoods (treating readings as independent via `np.prod`) and normalizes the result as shown:
```python
def update_step():
    for cur_x in range(mapper.MAX_CELLS_X):
        for cur_y in range(mapper.MAX_CELLS_Y):
            for cur_theta in range(mapper.MAX_CELLS_A):
                p = sensor_model(mapper.get_views(cur_x, cur_y, cur_theta))
                loc.bel[cur_x, cur_y, cur_theta] = np.prod(p) * loc.bel_bar[cur_x, cur_y, cur_theta]
    loc.bel = loc.bel / np.sum(loc.bel)
```

## Simulation Results
Running the preplanned trajectory, the raw odometry (red) drifted off the map within a few steps, while the Bayes-filtered belief (blue) tracked the ground truth (green) closely throughout.

![10_traj_Image](figures/10_lab/10_traj.png)
<div style="text-align: center;">
  <video width="640" height="480" controls>
    <source src="/figures/10_lab/10_bayes.mp4" type="video/mp4">
  </video>
</div>


#### Reflection
The filter is strongest near walls and corners, where the ToF readings return distinctive asymmetric range patterns. It weakens in open central regions of the environment where several candidate cells can produce nearly identical readings, so the belief stays diffuse and smears across 3 to 4 neighbors. Large pose jumps between timesteps also cause trouble: the `0.001` threshold in `prediction_step` can accidentally discard the previous cell if its prior belief was marginal, and the filter then needs a step or two of stronger sensor data to recover.

## Discussion
In this lab, I learned how what we used in ```Lab 9``` can be used to localize your robot in a given environment using the given beliefs and readings. I had some difficulty understanding the actual Bayes concept, but after reading through other students websites (And watching some youtube videos) I understoof what it was asking for. I look forward to applying this again in Lab 11 to realistically use a Bayes Filter in the real world.

#### Reference
I referred to Stephan Wagner's website for deciphering how the filter works.

[back](./)
````
