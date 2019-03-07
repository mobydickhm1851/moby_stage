moby_stage Working Diary
===

###### tags: `moby_stage`

> This file is to summary the unsolved problems and useful info for solabot simulation in stage. For progresses check out the [commit history][github_moby_stage].

[github_moby_stage]:https://github.com/mobydickhm1851/moby_stage

Unsolved Problems
---

### Look ahead dist setting now will cause __overlook__ at high speed
    - file : car0 and car1_linear_nav.py
    - line : 233
    - detailed info : 
        New look ahead dist should be used. 
    - ToDo :
        From a `point` to a `range`

### __Costmap is not shown__ 
    - file : car0 and car1_costmap_plot.py
    - line : 22
    - detailed info : 
        Originally the costmap was defined as 
        
            costmap = update.costmap
               
        where the value is 
        
            costmap = np.zeros((2,map_size/map_res, map_size/map_res), dtype=np.float32) 
             
        which doesn't show on plotting figure (the values are subscribed correctly).
        Then this line is change to the present form
        
            costmap=np.array(0.2*np.random.randn(2, map_size/map_res, map_size/map_res)+0.5, dtype=np.float32)
            
        where the only different is __initial value is not zero__, but it then plots the costmap normally.
    - guesses : 
        Might due to the method I used to chsnge the color setting which is 
        
            scat.set_array(zz) (line 69) 
        The function `set_array()` change the __color setting of the plot__.


### When the __size of the costmap is too large__ there will be delay for the velocity commands.
    - file : car0 and car1_linear_nav.py, multi_robots_intersection.world and multi_robots_intersection.launch
    - line : N/A
    - detailed info :
        As the size of the costmap exceed around 30 (actually it's relative to the velocity, kinda like reaction time is constant so the safe distance is relative to the approaching speed), the solabots will seem like neglecting the algorithm and drive directly into the other one. 
        However, as the velocity and the costmap size is correctly defined, they behave normally(stop right at the edge of the pixels where the `risk` equals to `1`).
        Also, when the t_lh or the costmap_size are too large, the claculation time will cause the delay of cmd_vels.
    - factors :
        - costmap_size
        - t_lh (look ahead time)
        - t_res (time resolution)
        - map_res (map resolution)
        - velocity
    - ToDo : 
        1. Reorganize the code and
        2. Made the costmap 'local costmap' (around the host agent only), so it can still work in larger maps.   


Useful Information
---

- When adding obstacles (more than one) to the simulation, here are what needed to be changed:
    1. obs_list 
    2. published topic name (in nav.py) and subscribed topic name (in plot.py)


- When the module `costmap_module` is imported, a new _process_ is created. In this case there are 4 nodes calling the module : car0_nav, car1_nav, car0_costmap_plot and car1_costmap_plot. Each of them brings up a `update` process which is not related to eachother. So, for example, if I want to update the parameters get from launch file and update them to `update` in the module imported, the `init_map()` should be called on each node.

