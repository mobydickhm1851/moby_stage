moby_stage Working Diary
===

###### tags: `moby_stage`

> This file is to summary the unsolved problems and useful info for solabot simulation in stage. For progresses check out the [commit history][github_moby_stage].

[github_moby_stage]:https://github.com/mobydickhm1851/moby_stage

Unsolved Problems
---

- __Costmap is not shown__ 
    - file : car0_costmap_plot.py (car1_costmap_plot.py)
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
