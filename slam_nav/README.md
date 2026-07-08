# Smart Cane RViz package

Files:
- smart_cane_viz_bridge.py: publishes current pose, traveled path, route relay and markers.
- start_smart_cane_viz_bridge.sh: starts the bridge on the RDK.
- smart_cane_remote.rviz: optional RViz preset for the external Ubuntu computer.

Expected topics:
- /map
- /tf and /tf_static
- /smart_cane/navigation_path OR /planned_landmark_path

Published topics:
- /smart_cane/current_pose
- /smart_cane/traveled_path
- /smart_cane/display_path
- /smart_cane/viz_markers
