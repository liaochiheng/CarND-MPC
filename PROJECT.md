# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Implementation

1. The Model: Global Kenematic Model
```
	x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    v_[t+1] = v[t] + a[t] * dt
    cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
    epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
 ```

 2. Timestep Length and Elapsed Duration (N & dt)

 	* dt: Generally, less dt, more frequent, and more accurate to fit the reference trajectory.
 	* N: The N*dt should be in few seconds, better in 2 seconds. Because the model is an approximation to the real dynamics, the more the N*dt is, the more the error is, and make Solver execution time longer.

 3. Polynomial Fitting and MPC Preprocessing

 	* Transform ptsx, ptsy into vehicle's coordinate system. ([main.cpp #142](src/main.cpp))
	 	```
	 	trans_points(next_x_vals, next_y_vals, state, ptsx, ptsy);
	 	```
	* Fit polynomial with order of 3
		```
		Eigen::VectorXd ref_ptsx = Eigen::VectorXd::Map( next_x_vals.data(), next_x_vals.size() );
        Eigen::VectorXd ref_ptsy = Eigen::VectorXd::Map( next_y_vals.data(), next_y_vals.size() );

	 	auto coeffs = polyfit(ref_ptsx, ref_ptsy, 3);
	 	```

 4. Model Predictive Control with Latency

 	* The latency is defined in [main.cpp #99](src/main.cpp).
	 	```
	 	mpc.latency = 100;
	 	```
 	* Deal with latency, will pass the predicted state after latency into Solve function. ([main.cpp #154](src/main.cpp))
 		```
 		Eigen::VectorXd state_vos(6); // vos = Vehicle's coOrdinate System
	    state_vos << 0, 0, 0, v, polyeval(coeffs, 0), epsi;
	    Eigen::VectorXd new_state_vos = mpc.StateAfterLatency(state_vos, delta, a);

	    vector<double> actuator = mpc.Solve(new_state_vos, coeffs);
 		```
 