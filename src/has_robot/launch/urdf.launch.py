<launch>
  <param name="robot_description" command="$(find xacro)/xacro --inorder $has_robot/urdf/has_robot.urdf.xacro" />

  <node name="urdf" pkg="urdf" type="urdf" />
</launch>