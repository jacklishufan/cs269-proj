###


Guide for Implementation


Generally, when add a task, we need to 
1. define an env (add objects)
2. write a "gold" human expert policy
3. Define Success Criterial

For 1., we need to add a task in `robosuite/environments/manipulation`. 
For example, I added `StackBook` and `MoveStackBook`.

For 2. Examples can be found in `robosuite/demos/new_task_shufan_1.py` and `robosuite/demos/new_task_shufan_2.py`. In these examples, we find the following line 
```
options["env_name"] = "StackBook"
```
and change the environment to newly created environment. We can run 

```
mjpython robosuite/demos/new_task_shufan_1.py
```

To visualize the policy. You may use standard python launcher instead of mjpython if you are not on a Mac.

For 3, this is the least concern, as we can manually label if a result is success or not for this course project. You can optionally implement an `is_success` function following the example of `mjpython robosuite/demos/new_task_shufan_1.py`, which checks 1) if objects are stacked 2) If large objects is are below small objects
