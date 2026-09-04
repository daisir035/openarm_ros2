# OpenArm joint slider panel

RViz2 panel for controlling both OpenArm arms and grippers through the existing
joint trajectory controllers. It follows `/joint_states`, reads limits from
`robot_description`, and sends large target changes as small trajectory steps.

Build and source the workspace, then launch the standard MoveIt fake-hardware
demo. The panel is included in the supplied RViz configuration, or it can be
added from `Panels -> Add New Panel -> openarmx_joint_slider_panel/JointSliderPanel`.

The package retains the source project's CC BY-NC-SA 4.0 license.
