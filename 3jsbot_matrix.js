//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

/**
 * Retrieves the size of the matrix
 */
function get_matrix_size(matrix) {
	if (typeof(matrix)==='object' || typeof(matrix)==='array') {
		nrow = matrix.length;
		if (matrix[0]) {
			if (typeof(matrix[0])==='object' || typeof(matrix[0])==='array')
				ncol = matrix[0].length;
			else
				ncol = 1;
		} else
			ncol = 0;
		return [nrow, ncol];
	} else
		return null;
}

/**
 * Multiplies the two matrices
 */
function matrix_multiply(mata, matb) {
	size_a = get_matrix_size(mata);
	size_b = get_matrix_size(matb);
	result = []; 
	if (!size_a || !size_b)
		return null;
	if (size_a[1] == size_b[0] && size_a[1] != 0) {
		for (var i=0; i<size_a[0]; i++) {
			result[i] = [];
			for (var j=0; j<size_b[1]; j++) {
				result[i][j] = 0;
				for (var k=0; k<size_a[1]; k++)
					result[i][j] += mata[i][k] * matb[k][j];
			}
		}
		return result;
	} else
		return null;
}

/**
 * Calculates the transpose of a matrix
 */
function matrix_transpose(mat) {
	size = get_matrix_size(mat);
	if (!size)
		return null;
	result = [];
	for (var j=0; j<size[1]; j++)
		result[j] = [];
	for (var i=0; i<size[0]; i++)
		for (var j=0; j<size[1]; j++)
			result[j][i] = mat[i][j];
	return result;
}

/**
 * Normalizes a vector
 */
function vector_normalize(vec) {
	newvec = [];
	sum = 0;
	for (i=0; i<vec.length; i++) {
		sum = sum + vec[i] * vec[i];
	}
	sqrt = Math.sqrt(sum);
	for (i=0; i<vec.length; i++) {
		newvec.push(vec[i] / sqrt);
	}
	return newvec
	// return numeric.div(vec, Math.sqrt(numeric.sum(numeric.dot(vec, vec))));
}

function vector_mag(vec) {
	sum = 0;
	for (i=0; i<vec.length; i++) {
		sum = sum + vec[i] * vec[i];
	}
	return Math.sqrt(sum);
}
/**
 * retrieves a cross vector from two vectors
 */
function vector_cross(veca, vecb) {
	if (veca.length >= 3 && vecb.length >=3) {
		result = []
		result[0] = veca[1] * vecb[2] - veca[2] * vecb[1];
		result[1] = veca[2] * vecb[0] - veca[0] * vecb[2];
		result[2] = veca[0] * vecb[1] - veca[1] * vecb[0];
		// for vectors in homogeneous coordinates
		if (veca.length > 3)
			result[3] = 1
		return result;
	} else
		return null;
}

/**
 * retrieves an indentity matrix
 */
function generate_identity(n) {
	if (typeof(n) === 'undefined')
		n = 4;
	matrix = []
	for (i = 0; i < n; i++) {
		matrix[i] = [];
		for (j = 0; j < n; j++)
			matrix[i][j] = (i==j ? 1 : 0);
	}
	return matrix;
}

/**
 * retrieves a translation matrix given x, y, z coordinates
 */
function generate_translation_matrix(xyz) {
	return [[1, 0, 0, xyz[0]],
				  [0, 1, 0, xyz[1]],
				  [0, 0, 1, xyz[2]],
				  [0, 0, 0, 1]];
}

function generate_translation_matrix_from_vector_matrix(xyz) {
	return [[1, 0, 0, xyz[0][0]],
				  [0, 1, 0, xyz[1][0]],
				  [0, 0, 1, xyz[2][0]],
				  [0, 0, 0, 1]];
}

/**
 * retrieves a rotation matrix in x axis given x theta in radian
 */
function generate_rotation_matrix_X(theta) {
	return [[1,               0,                0, 0],
				  [0, Math.cos(theta), -Math.sin(theta), 0],
				  [0, Math.sin(theta),  Math.cos(theta), 0],
				  [0,               0,                0, 1]];
}

/**
 * retrieves a rotation matrix in y axis given y theta in radian
 */
function generate_rotation_matrix_Y(theta) {
	return [[ Math.cos(theta), 0, Math.sin(theta), 0],
				  [               0, 1,               0, 0],
				  [-Math.sin(theta), 0, Math.cos(theta), 0],
				  [               0, 0,               0, 1]];
}

/**
 * retrieves a rotation matrix in z axis given z theta in radian
 */
function generate_rotation_matrix_Z(theta) {
	return [[Math.cos(theta), -Math.sin(theta), 0, 0],
				  [Math.sin(theta),  Math.cos(theta), 0, 0],
				  [              0,                0, 1, 0],
				  [              0,                0, 0, 1]];
}
