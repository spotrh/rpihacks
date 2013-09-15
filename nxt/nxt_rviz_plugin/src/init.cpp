#include <pluginlib/class_list_macros.h>

#include "nxt_ultrasonic_display.h"
#include "nxt_color_display.h"

PLUGINLIB_DECLARE_CLASS( rviz_qt, NXTUltrasonicDisplay, nxt_rviz_plugin::NXTUltrasonicDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, NXTColorDisplay, nxt_rviz_plugin::NXTColorDisplay, rviz::Display )

