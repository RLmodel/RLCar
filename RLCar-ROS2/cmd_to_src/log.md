1. DC 모터 pwm 제어

pwm : 실제 속도
112 | 0.125
120 | 0.3125
136 | 0.5
152 | 0.75
177 | 1.0
192 | 1.25
209 | 1.5
225 | 1.75
255 | 2.0

첫 시작 시 0부터 시작하면 0.125까지 너무 오래 걸린다.
따라서 가속이 시작되면 pwm 100부터 시작되도록 해두었음

2. encoder값과 속도 제어

encoder 1당 움직이는 거리 : 0.0508 * (2pi / 150) - 150은 엔코더가 한바퀴 도는 데 걸리는 시간
encoder 1당 속도 (50Hz 0.02초 기준) :  (0.0508 * (2pi / 150)) / 0.02 - 약 0.10639
따라서, 1m/s가 되기 위해서는 0.02초당 약 9개 엔코더 변화가 있어야 한다. (9.39891)
현재 scale이 그래서 9가 되어있는 것임

실제 돌려본 결과
0.125 m/s => *9 => 1.125 => int casting => 1 따라서, odom을 통해 측정한 속도도 0.105정도이다.

속도 분해능이 약 0.10639가 된 것임

3. 조향각

필요한 것들만 정리해보자.

```
vel_base_link = self.linear_velocity = data.linear.x

omega_base_link = self.omega_turning_speed
self.omega_turning_speed = self.linear_velocity / self.steering_radius
self.steering_radius = max(abs(steering_radius_raw), self.R_Min_baselink)
steering_radius_raw = abs(self.linear_velocity / data.angular.z)

self.maxsteerInside = max_abs_steer
R_Min_interior = self.L / math.tan(self.maxsteerInside)
self.R_Min_baselink = R_Min_interior + (self.T / 2.0)

alfa_right_front_wheel = math.asin((omega_base_link * self.L ) / vel_base_link)
```

필요한 값
maxsteerInside : 
T : 
L : 

```python
it = 0
while sum(dp) > tol:
    # print("Iteration {}, best error = {}".format(it, best_err))
    for i in range(len(p)):
        p[i] += dp[i]
        robot = make_robot()
        x_trajectory, y_trajectory, err = run(robot, p)

        if err < best_err:
            best_err = err
            dp[i] *= 1.1
        else:
            p[i] -= 2 * dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] += dp[i]
                dp[i] *= 0.9
```

gain tuning

P : 100 - 120 