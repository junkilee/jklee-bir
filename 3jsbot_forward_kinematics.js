//////////////////////////////////////////////////
/////     FORWARD KINEMATICS
//////////////////////////////////////////////////

// CS148: compute and draw robot kinematics (.xform matrix for each link)
// CS148: compute and draw robot heading and lateral vectors for base movement in plane
// matrix_2Darray_to_threejs converts a 2D JavaScript array to a threejs matrix
//   for example: var tempmat = matrix_2Darray_to_threejs(link.xform);
// simpleApplyMatrix transforms a threejs object by a matrix
//   for example: simpleApplyMatrix(link.geom,tempmat);

/*
CS148: reference code has functions for:

robot_forward_kinematics
traverse_forward_kinematics_link
traverse_forward_kinematics_joint
compute_and_draw_heading
*/

/**
 * retrieves a rotation matrix for rpy (x_theta, y_theta, z_theta)
 */
function generate_rotation_matrix(rpy) {
	if (typeof(rpy) === 'object') {
		matrix = generate_rotation_matrix_X(rpy[0]);
		matrix = matrix_multiply(matrix, generate_rotation_matrix_Y(rpy[1]));
		return matrix_multiply(matrix, generate_rotation_matrix_Z(rpy[2]));
	} else
		return null;
}

/**
 * calculates tranformation matrices for the entire robot model
 */
function robot_forward_kinematics() {
	var heading_local = [[1], [0], [0], [1]];
	var lateral_local = [[0], [0], [1], [1]];

	robot.origin.xform = generate_translation_matrix(robot.origin.xyz);
	robot.origin.xform = matrix_multiply(robot.origin.xform, generate_rotation_matrix(robot.origin.rpy));

	robot_heading = matrix_multiply(robot.origin.xform, heading_local);
	heading_mat = matrix_2Darray_to_threejs(generate_translation_matrix_from_vector_matrix(robot_heading));

	robot_lateral = matrix_multiply(robot.origin.xform, lateral_local);
	lateral_mat = matrix_2Darray_to_threejs(generate_translation_matrix_from_vector_matrix(robot_lateral));

	simpleApplyMatrix(heading_geom,heading_mat);
  simpleApplyMatrix(lateral_geom,lateral_mat);

	traverse_forward_kinematics_link(robot.base);
}

/**
 * calculates tranformation matrices for each link
 */
function traverse_forward_kinematics_link(link) {
	if (robot.links[link].parent) {
		robot.links[link].xform = robot.joints[robot.links[link].parent].xform;
	} else {
		robot.links[link].xform = robot.origin.xform;		
	}
	
	var tempmat = matrix_2Darray_to_threejs(robot.links[link].xform);
  simpleApplyMatrix(robot.links[link].geom, tempmat);

  if (robot.links[link].children) {
  	for (i in robot.links[link].children) {
  		traverse_forward_kinematics_joint(robot.links[link].children[i], robot.links[link].xform);
  	}
  }  
}

/**
 * calculates tranformation matrices for each joint
 */
function traverse_forward_kinematics_joint(joint) {
	parent_link = robot.joints[joint].parent;
	robot.joints[joint].origin.xform = matrix_multiply(robot.links[parent_link].xform, 
																										 generate_translation_matrix(robot.joints[joint].origin.xyz));
	robot.joints[joint].origin.xform = matrix_multiply(robot.joints[joint].origin.xform, 
																										 generate_rotation_matrix(robot.joints[joint].origin.rpy));
	
	/* applying joint angles */
	q = quaternion_from_axisangle(robot.joints[joint].axis, robot.joints[joint].angle);
	rotq = quaternion_to_rotation_matrix(q);
	robot.joints[joint].xform = matrix_multiply(robot.joints[joint].origin.xform, rotq);
	var tempmat = matrix_2Darray_to_threejs(robot.joints[joint].xform);
  simpleApplyMatrix(robot.joints[joint].geom, tempmat);

  if (robot.joints[joint].child)
  	traverse_forward_kinematics_link(robot.joints[joint].child);
}

function compute_and_draw_heading() {
}