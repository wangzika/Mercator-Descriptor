

# The Mercator Descriptor: A Novel Global Descriptor for Loop Closure Detection

Welcome to the Mercator Descriptor repository! The Mercator Descriptor is a novel descriptor for loop closure detection and place recognition. This descriptor first uses a Mercator-like projection to generate a rotation-invariant descriptor from a frame of point clouds. Then, by constructing a database to store this new descriptor, it enables efficient matching between historical and current frames for loop closure detection.
![](https://github.com/wangzika/Mercator-Descriptor/blob/main/Graphical%20abstract.jpg)


## Examples 


video: [Youtube link]([https://youtu.be/W5HYYPYBrn8](https://www.youtube.com/watch?v=uXGNIHsfkAw))


### ** Example-1: place recognition with KITTI Odometry dataset**
<div align="center">
<img src="https://github.com/wangzika/Mercator-Descriptor/blob/main/loop%20detection1.GIF"  width="48%" loop/>
<img src="https://github.com/wangzika/Mercator-Descriptor/blob/main/loop%20detection.GIF"  width="48%" loop/>
</div>



### ** Example-2: loop closure correction on our dataset**
<div align="center">
<img src="https://github.com/wangzika/Mercator-Descriptor/blob/main/captum_r.jpg"  width="80%" loop/>
<img src="https://github.com/wangzika/Mercator-Descriptor/blob/main/captum_lc.jpg"  width="80%" loop/>
</div>
The figure depicts an overview of the Backpack laser scanning system and the campus dataset, as well as the resulting graphs with and without loop closure.

# Developers:
Wang Zhibo(e-mail:582796566@qq.com)


# Credits

We hereby recommend reading [Scan Context](https://github.com/gisbi-kim/scancontext_tro) ,[FAST_LIO](https://github.com/hku-mars/FAST_LIO) ,[A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and [STD](https://github.com/hku-mars/STD) for reference and thank them for making their work public.

# License

The source code is released under GPLv3 license.

I am constantly working on improving this code. For any technical issues or commercial use, please contact me(582796566@qq.com).

