# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


## Project Rubric

### Implementation
In this project I have implemented a cascaded PID controller to control the steering and the throttle of the vehicle. The first PID is designed to calculate the steering value to minimize the CTE (cross-track error). The second PID controller controls the speed of the car in porportion to the steering value.
The PID implementation is done on the ./src/PID.cpp. The PID::UpdateError method calculates proportional, integral and derivative errors and the PID::TotalError calculates the total error using the appropriate coefficients.

## Reflections
- The "P" for proportional means that the car will steer in proportion to the cross-track error, or CTE. CTE is essentially how far from the middle line of the road the car is. If the proportional gain is set too low the controller will be unresponsive to the error and if set too high the controller will overshoot the desired value and the controller tends to over correct itself. this constant cycle of over-shoots and over corrections result in oscillations.

- The "I" for integral sums up all CTEs up to that point, such that too many negative CTEs (in this case, meaning the car has been to the left of the middle of the lane for awhile) will drive up this value, causing the car to turn back toward the middle, preventing the car from driving on one side of the lane the whole time. If the coefficient is too high for I, the car tends to have quicker oscillations, and does not tend to get up to a quick speed. A low coefficent for I will cause the car to tend to drift to one side of the lane or the other for longer periods of time.

- The differential portion helps to counteract the proportional trend to overshoot the center line by smoothing the approach to it.
This means that 1) if the derivative is quickly changing, the car will correct itself (i.e. higher steering angle) faster, such as in the case of a curve, and 2) if the car is moving outward from the middle, this will cause the steering to get larger (as the derivative sign will match the proportional sign), but if the car is moving toward the center (meaning the derivative value will be negative), the car's steering angle will get smoothed out, leading to a more smoother driving experience. 

### Finding the right gains
Although it is removed from the final code, I had used Twiddle a little bit to try out different parameters, but found the values found in the original project lessons to be sufficient under my current implementation. I tried dynamically tuning the PID gains using Twiddle algorithm but later found out it was not performing well, as the vehicle sometimes behaved erratically. So I decided to go with the traditional manual tuning method to tune my PID gains. The gains for both the controller (steering + speed) were tuned such the the vehicle performed well on both larger turns aswell as sharp turns. The final parameters for the steering controller and the speed controller where [P: 0.15, I: 0.0, D: 2.5] and [P: 0.8, I: 0.0, D: 2.5] respectively.
