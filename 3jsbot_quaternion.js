//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

/*
CS148: reference code has functions for:

quaternion_from_axisangle
quaternion_normalize
quaternion_multiply
quaternion_to_rotation_matrix
*/

function quaternion_from_axisangle(axis, theta) {
	res = [];
	res[0] = Math.cos(theta/2.0);
	res[1] = Math.sin(theta/2.0) * axis[0];
	res[2] = Math.sin(theta/2.0) * axis[1];
	res[3] = Math.sin(theta/2.0) * axis[2];
	return res;
}

function quaternion_normalize(q) {
	res = [];
	n = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
	res[0] = q[0] / Math.sqrt(n);
	res[1] = q[1] / Math.sqrt(n);
	res[2] = q[2] / Math.sqrt(n);
	res[3] = q[3] / Math.sqrt(n);
	return res;
}

function quaternion_multiply(q, r) {
	res = [];
	res[0] = r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3];
	res[1] = r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2];
	res[2] = r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1];
	res[3] = r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0];
	return res;
}

function quaternion_to_rotation_matrix(q) {
	q = quaternion_normalize(q);
	return [[1 - 2 * (q[2] * q[2] + q[3] * q[3]), 2 * (q[1] * q[2] - q[0] * q[3]), 2 * (q[0] * q[2] + q[1] * q[3]), 0],
					[2 * (q[1] * q[2] + q[0] * q[3]), 1 - 2 * (q[1] * q[1] + q[3] * q[3]), 2 * (q[2] * q[3] - q[0] * q[1]), 0],
					[2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]), 0],
					[0, 0, 0, 1]];
}