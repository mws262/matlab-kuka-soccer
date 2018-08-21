## Tests which exercise various parts of the code.
Note that these are only tests in an informal sense. For me to tell if things look right.

### test_nlp_spline
Basic test of the path spline maker. Optimizes to try to make more linear sections.

### test_qp_nlp_spline
More complete test of the spline makers. Does simpler version as QP, uses this as initial guess for NLP, then shifts it around so it begins with a contact section.

### test_transform_link_to_touch_surface
Specify a point and vector to make some spot on the "dummy" link tangent to. Rotates it about this axis, keeping it tangent. Just to make sure geometry is working ok.

### test_integrate_velocity_over_surface
Given a contact velocity, integrate this path over the surface of the "dummy" link. This is done on 3 different resolution models of the foot link.

### test_mesh_on_ball_contact
Integrate contact velocity over the dummy link and make it "roll" over the surface of the ball kinematically as the ball moves along a path.

### test_ik_goal
Get a point on the foot to a point in space as best as possible. Just to test IK and also geometry of the link's origin and transforms vs. some surface point.

### test mesh_on_ball_optim
Optimize initial contact location on dummy foot, initial contact angle, and initial angle along the ball arc so that the entire contact section is kinematically possible (or as good as we can find).

### test_mesh_on_ball_multiseg_optim
Same as test_mesh_on_ball_optim, but with multiple sections. IK for connecting these is still in progress.

### test_mesh_normals
Can load any number of meshes from get_mesh_data and display their face and vertex normals. Sometimes models have them reversed.

### test_quatintegrate
Make sure that the RK4 quaternion integrator works ok relative to the old dumb euler.

### test_banned_regions
Shows edited "dummy" link and the convex shape which defines areas not allowed for ball contact.
Points are sampled on the surface and are red if they fall in the banned region, green if they are not.

### test_mark_on_mesh
Tests graphically marking points on the mesh nearest to some point.

