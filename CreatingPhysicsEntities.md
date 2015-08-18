# Instructions #
  1. Create a brush which will become the physics entity.
  1. Create a second brush, covered with the system/origin shader, at the _very center_ of the first brush. This is crucial in getting the brush to act physically correct.
  1. Select the origin brush, and then the first brush (CTRL + Shift), and turn them both into a func\_physics entity.
  1. Press n to bring up the Entity Properties window.
  1. Change the _shape_ key to fit the physics object. Only use the _custom_ shape if no other shape fits your object.

> The table below shows the available shapes:

| **Number** | **Associated Shape** |
|:-----------|:---------------------|
| 0          | Cuboid (all objects are cuboids by default) |
| 1          | Sphere               |
| 2          | Cylinder             |
| 3          | Capsule              |
| 4          | Custom shape         |