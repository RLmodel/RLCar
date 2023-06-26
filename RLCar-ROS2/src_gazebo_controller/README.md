# Ackermann Steering Car Controller dynamics

> Convert `cmd_vel` type msg to `Ackermann Steering Control signal`

Ackermann Steering Robot in this example consists of...
- Each Wheel can controlled separately (So it means differential gear in real.)
- Front Wheels can steered with revolute hinge joints 

## Step 1. Let's start from basic bycicle model dynamics

We can calculate steering radius && wheel rotational velocity for COM

![슬라이드1](https://user-images.githubusercontent.com/12381733/164611080-19371a04-c7f8-417a-b724-9109d38673a4.png)

## Step 2. Calculate rear wheel velocity with different steering radius

![슬라이드2](https://user-images.githubusercontent.com/12381733/164611083-084b131a-bb18-4214-860c-d979f81aab1e.png)

> Care for combination of linear & angular velocity direction. If you confused, refer below images

![슬라이드4](https://user-images.githubusercontent.com/12381733/164611089-400f1e29-3b77-4bee-9a2e-2a07df9b4146.png)

## Step 3. Calculate front wheel velocity & steering angle alpha for each joints

There's lots of equations for this `alpha`. I tried all of them and found best one empirically.

![슬라이드3](https://user-images.githubusercontent.com/12381733/164611086-037f300f-6ab8-4a15-b753-d26ba9e25eed.png)


