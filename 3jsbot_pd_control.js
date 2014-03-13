//////////////////////////////////////////////////
/////     MOTION CONTROL ROUTINES 
//////////////////////////////////////////////////

// CS148: add PD controller here
function robot_pd_control () {
	var curdate = new Date();
	var for_active_joint = false;
	
	// for just the active joint
	if (for_active_joint && active_joint) {
		robot.joints[active_joint].servo.p_desired = curdate.getSeconds() / 60 * 2 * Math.PI;
		if (robot.joints[active_joint].servo.p_desired - robot.joints[active_joint].angle < -Math.PI) {
			robot.joints[active_joint].servo.p_desired += 2 * Math.PI;
		}
		robot.joints[active_joint].control = (robot.joints[active_joint].servo.p_desired - robot.joints[active_joint].angle) 
		                                     * robot.joints[active_joint].servo.p_gain;
	}

	// for all joints
	if (!for_active_joint) {
		for (x in robot.joints) {
			robot.joints[x].servo.p_desired = curdate.getSeconds() / 60 * 2 * Math.PI;
			if (robot.joints[x].servo.p_desired - robot.joints[x].angle < -Math.PI) {
				robot.joints[x].servo.p_desired += 2 * Math.PI;
			}
			robot.joints[x].control = (robot.joints[x].servo.p_desired - robot.joints[x].angle) * robot.joints[x].servo.p_gain;
		}
	}
}