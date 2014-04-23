//////////////////////////////////////////////////
/////     INVERSE KINEMATICS 
/////     Resolved-rate IK with geometric jacobian
//////////////////////////////////////////////////

// CS148: generate joint controls to move robot to move robot endeffector to target location

/*
CS148: reference code has functions for:

robot_inverse_kinematics
iterate_inverse_kinematics
*/
function robot_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {
// compute joint angle controls to move location on specified link to Cartesian location
    if (update_ik) {
        iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos);
        endeffector_geom.visible = true;
        target_geom.visible = true;
    } else {
        endeffector_geom.visible = false;
        target_geom.visible = false;
    }
    update_ik = false;
}

function iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {
    var joints_list = [];
    target_pos_ = target_pos;
    current_joint = endeffector_joint
    while (current_joint != '') {
        joints_list.unshift(current_joint)
        if (robot.joints[current_joint].parent != robot.base) {
            current_joint = robot.links[robot.joints[current_joint].parent].parent;
        } else {
            current_joint = '';
        }
    }

    // start from robot.origin.xform

    current_translation_matrix = generate_identity(4);
    // z^0_i-1 = R^0_i-1 * q
    // T^0_n (q) = [R^0_n(q) o^0_n(q)]
    //             [    0       1    ]
    // 
    // retrieve the forward kinematic chain
    // Ji = [ z_i-1 X (o_n - o_i-1) ] from 1 to n
    //      [ z_i-1                 ]

    // and calculate z_i-1 (axis), o_n (points), o_i-1 (points, origin of joins)
    transforms = [robot.origin.xform, robot.origin.xform];

    for (i in joints_list) {
        index = Number(i) + 1;
        joint = joints_list[i];
        //before rotation
        transforms[index] = matrix_multiply(transforms[index], generate_translation_matrix(robot.joints[joint].origin.xyz));
        transforms[index] = matrix_multiply(transforms[index], generate_rotation_matrix(robot.joints[joint].origin.rpy));
        
        //after rotation
        q = quaternion_from_axisangle(robot.joints[joint].axis, robot.joints[joint].angle);
        rotq = quaternion_to_rotation_matrix(q);

        transforms[index + 1] = matrix_multiply(transforms[index], rotq)
    }

    n = index + 1;
    transforms[n] = matrix_multiply(transforms[n], generate_translation_matrix(endeffector_local_pos));

    jacobian = [[], [], [], [], [], []];
    
    o_n = get_origin(transforms[n]);


    for (i in transforms.slice(1, transforms.length - 1)) {
        joint = joints_list[Number(i)];
        z = apply_tranform_to_axis(transforms[Number(i) + 1], robot.joints[joint].axis);
        delta_o = numeric.sub(o_n, get_origin(transforms[Number(i) + 1]));
        j_v = vector_cross(z, delta_o);
        add_j_v_j_w_to_jacobian(jacobian, i, j_v, z);
    }

    // x_n = apply_tranform_to_axis(transforms[n], [1,0,0]);
    // y_n = apply_tranform_to_axis(transforms[n], [0,1,0]);
    // z_n = apply_tranform_to_axis(transforms[n], [0,0,1]);
    // w_x = get_theta_from_vectors([1, 0, 0], x_n);
    // w_y = get_theta_from_vectors([0, 1, 0], y_n);
    // w_z = get_theta_from_vectors([0, 0, 1], z_n);
    w_x = 0;
    w_y = 0;
    w_z = 0;

    delta_x = [[target_pos[0] - o_n[0]], [target_pos[1] - o_n[1]], [target_pos[2] - o_n[2]], [w_x], [w_y], [w_z]];

    use_pseduo_inverse = true;

    if (!use_pseduo_inverse) {
        delta_q = matrix_multiply(numeric.transpose(jacobian), delta_x);
    } else { 
        j_size = get_matrix_size(jacobian);
        if (j_size[0] > j_size[1]) {
            delta_q = matrix_multiply(get_left_pseudo_inverse(jacobian), delta_x);
        } else {
            delta_q = matrix_multiply(get_right_pseudo_inverse(jacobian), delta_x);
        }
    }
    

    alpha = 0.1;

    for (i in joints_list) {
        joint = joints_list[i];
        robot.joints[joint].control = alpha * delta_q[i][0];
    }
    
    // draw endeffector and target position indicators
    var endeffector_mat = matrix_2Darray_to_threejs(transforms[n]);
    simpleApplyMatrix(endeffector_geom,endeffector_mat);
   
    if (navigator.userAgent.indexOf("Firefox")!=-1) 
        var target_mat = matrix_2Darray_to_threejs(generate_translation_matrix_from_vector_matrix(target_pos_));
    else
        var target_mat = matrix_2Darray_to_threejs(generate_translation_matrix(target_pos_));
    simpleApplyMatrix(target_geom,target_mat);
}

function get_theta_from_vectors(a, b) {
    return Math.acos(numeric.dot(a, b) / (Math.sqrt(numeric.dot(a,a)) * Math.sqrt(numeric.dot(b,b))));
}

function apply_tranform_to_axis(transform, axis) {
    return get_vector_from_transposed_matrix(
                matrix_multiply(
                    get_rotation_matrix(transform),
                    get_matrix_transpose_from_vector(axis)));
}

function get_left_pseudo_inverse(matrix) {
    inverse = numeric.inv(numeric.dot(numeric.transpose(matrix), matrix));
    return numeric.dot(inverse, numeric.transpose(matrix));
}

function get_right_pseudo_inverse(matrix) {
    inverse = numeric.inv(numeric.dot(matrix, numeric.transpose(matrix)));
    return numeric.dot(numeric.transpose(matrix), inverse);
}

function add_j_v_j_w_to_jacobian(jacobian, index, j_v, j_w) {
    jacobian[0][index] = j_v[0];
    jacobian[1][index] = j_v[1];
    jacobian[2][index] = j_v[2];
    jacobian[3][index] = j_w[0];
    jacobian[4][index] = j_w[1];
    jacobian[5][index] = j_w[2];
}

function get_rotation_matrix(transf) {
    return [[transf[0][0], transf[0][1], transf[0][2]],
            [transf[1][0], transf[1][1], transf[1][2]],
            [transf[2][0], transf[2][1], transf[2][2]]];
}

function get_origin(transf) {
    return [transf[0][3], transf[1][3], transf[2][3]];
}

function get_vector_from_transposed_matrix(matrix) {
    vec = [];
    for (e in matrix) {
        vec[e] = matrix[e][0];
    }
    return vec;
}

function get_matrix_transpose_from_vector(vec) {
    mat = [];
    for (e in vec) {
        mat[e] = [vec[e]];
    }
    return mat;
}
