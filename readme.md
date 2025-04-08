1. 检测插件正确安装
```cpp
ls  ./devel/lib/ |grep astar_planner
libastar_planner.so
```
2. 修改`nav_path.launch`
```json
        <!-- 指定全局路径规划器 -->
        <param name="base_global_planner" type="string" value="custom_global_planner/AstarPlanner" />

        <!-- 加载全局路径规划器的参数文件（如果有的话） -->
        <rosparam file="$(find custom_navigation)/astar/param/global_planner_params.yaml" command="load"/>

        <!-- 指定局部路径规划器 -->
        <param name="base_local_planner" type="string" value="custom_local_planner/DWAPlanner" />

        <!-- 加载局部路径规划器的参数文件 -->
        <rosparam file="$(find custom_navigation)/dwa/param/local_planner_params.yaml" command="load" />
```
3. 验证是否正确调用，通过topic验证。

Reference:https://github.com/davidezilio/custom_navigation