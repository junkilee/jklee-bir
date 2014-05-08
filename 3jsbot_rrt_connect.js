//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// CS148: 
// implement RRT-Connect by Kuffner and LaValle (2000)
//    paper link: http://msl.cs.uiuc.edu/~lavalle/papers/KufLav00.pdf

// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by robot_collision_test()

start_tree = null;
goal_tree = null;
tree_a = null;
tree_b = null;
rrt_epsilon_pos = 0.2;
rrt_epsilon_ang = 0.2;

/**
 * Initializes the RRT planner
 */
function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // delete the geoms from the previous planned route
    if (start_tree != null) {
        for (i in start_tree.vertices) {
            scene.remove(start_tree.vertices[i].geom);
        }
        for (i in goal_tree.vertices) {
            scene.remove(goal_tree.vertices[i].geom);
        }
    }

    // initialize all the trees
    start_tree = tree_init(q_start_config);
    goal_tree = tree_init(q_goal_config);    
    tree_a = start_tree;
    tree_b = goal_tree;
    robot_path = [];

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    console.log("planner initialized");
}

iteration = 0;
/**
 * RRT Connect iteration function
 **/
function robot_rrt_planner_iterate() {

    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)
    iteration++;

    rrt_iterate = true
    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

        if (rrt_alg == 1) {
            q_rand = random_config();
            rrr = rrt_extend(tree_a, q_rand);
            if (rrr != 'trapped') {
                if (rrt_connect(tree_b, tree_a.vertices[tree_a.newest].vertex) == 'reached') {
                    robot_path = find_path();
                    for (i in robot_path)
                        robot_path[i].geom.material.color = {r:1,g:0,b:0};
                    return 'reached';
                }
            }
            swap_tree();
        } else {
            // not implemented yet.
        }
    }

    // return path not currently found
    return false;
}

function random_config() {
    q = [];
    for (i=0; i<3; i++) {
        q[i] = Math.random() * (robot_boundary[1][i] - robot_boundary[0][i]) + robot_boundary[0][i];
    }
    q[3] = 0;
    q[4] = Math.random() * 2 * Math.PI;
    q[5] = 0;
    for (i = 6; i<q_start_config.length; i++)
        q[i] = Math.random() * 2 * Math.PI;
    return q;
}

/**
 * calculates epsilons for new_config function
 */
function get_epsilon(number, ispos) {
    if (ispos && (number < rrt_epsilon_pos && number > -rrt_epsilon_pos))
        return number;
    if (!ispos && (number < rrt_epsilon_ang && number > -rrt_epsilon_ang))
        return number;
    if (ispos)
        return number<0.0?-rrt_epsilon_pos:rrt_epsilon_pos;
    else
        return number<0.0?-rrt_epsilon_ang:rrt_epsilon_ang;
}

function new_config(q_near, q) {
    var q_new = [];
    var q_diff1 = [];
    var q_diff2 = [];

    for (i=0; i<3; i++) {
        q_diff1[i] = q[i] - q_near[i];
    }
    for (i=3; i<q_near.length; i++) {
        q_diff2.push(q[i] - q_near[i]);
    }

    var q_norm_diff1 = vector_normalize(q_diff1);
    var q_norm_diff2 = vector_normalize(q_diff2);

    if (vector_mag(q_diff1) <= rrt_epsilon_pos) {
        for (i=0; i<3; i++) {
            q_new[i] = q_near[i] + q_diff1[i];
        }
    } else {
        for (i=0; i<3; i++) {
            q_new[i] = q_near[i] + q_norm_diff1[i] * rrt_epsilon_pos;
        }
    }

    if (vector_mag(q_diff2) <= rrt_epsilon_ang) {
        for (i=3; i<q_near.length; i++) {
            q_new[i] = q_near[i] + q_diff2[i-3];
        }
    } else {
        for (i=3; i<q_near.length; i++) {
            q_new[i] = q_near[i] + q_norm_diff2[i-3] * rrt_epsilon_ang;
        }
    }
    console.log("new one created..");

    // new way of implmenting this
    // get a difference vector for both position and angles and make it as a unit vector and multiply with epsilon.

    if (robot_collision_test(q_new))
        return null;
    return q_new;
}

function nearest_neighbor(tree, q) {
    dist1 = Number.MAX_VALUE;
    dist2 = Number.MAX_VALUE;
    nearest = tree.newest;
    //check the newest first
    if (distance_check(tree.vertices[tree.newest].vertex, q)) {
        return tree.newest;
    }
    
    // check other tree nodes
    // there are two criteria for calculating the nearest tree node
    // (1) x,y,z distance
    // (2) base rotation and angle rotations
    for (i in tree.vertices) {
        sub1 = numeric.sub(tree.vertices[i].vertex.slice(0,3), q.slice(0,3));
        sub2 = numeric.sub(tree.vertices[i].vertex.slice(3), q.slice(3));
        value1 = Math.sqrt(numeric.dot(sub1,sub1));
        value2 = Math.sqrt(numeric.dot(sub2,sub2));
        if (value1 < dist1) {
            dist1 = value1;
            dist2 = value2;
            nearest = i;
        } else if (value1 - dist1 < rrt_epsilon_pos * 0.33 ) {
            if (value2 < dist2) {
                dist1 = value1;
                dist2 = value2;
                nearest = i;
            }
        }
    }
    return nearest;
}

/**
 * checks if the destination is close so that it can be directly picked as a neighbor
 */
function distance_check(q_new, q) {
    var q_diff1 = [];
    var q_diff2 = [];

    for (i=0; i<3; i++) {
        q_diff1[i] = q_new[i] - q[i];
    }
    for (i=3; i<q_new.length; i++) {
        q_diff2.push(q_new[i] - q[i]);
    }

    if ((vector_mag(q_diff1) < rrt_epsilon_pos * 1.5) && (vector_mag(q_diff2) < rrt_epsilon_ang * 1.5)) {
        return true;
    }
    return false;
}

function reach_check(q_new, q) {
    var q_diff1 = [];
    var q_diff2 = [];

    for (i=0; i<3; i++) {
        q_diff1[i] = q_new[i] - q[i];
    }
    for (i=3; i<q_new.length; i++) {
        q_diff2.push(q_new[i] - q[i]);
    }

    if ((vector_mag(q_diff1) < rrt_epsilon_pos) && (vector_mag(q_diff2) < rrt_epsilon_ang)) {
        return true;
    }
    return false;
}

function rrt_extend(tree, q) {
    q_near_index = nearest_neighbor(tree, q)
    q_new = new_config(tree.vertices[q_near_index].vertex, q);
    if (q_new !== null) {
        q_new_index = tree_add_vertex(tree, q_new);
        tree_add_edge(tree, q_near_index, q_new_index);
        if (reach_check(tree.vertices[q_new_index].vertex, q)) {
            return 'reached';
        } else {
            return 'advanced';
        }
    }
    console.log("trapped!!!!!");
    return 'trapped';
}

function rrt_connect(tree, q_index) {
    s = 'advanced';
    while (s == 'advanced') {
        s = rrt_extend(tree, q_index);
        console.log('connecting..   ' + s);
    }
    return s;
}

/**
 * swaps tree_a and tree_b
 */
function swap_tree() {
    var tree_temp = tree_a;
    tree_a = tree_b;
    tree_b = tree_temp;
}

/**
 * calculates the path
 */
function find_path() {
    if (tree_a != start_tree)
        swap_tree();
    r1 = tree_backtrack(tree_a, tree_a.newest, true);
    r2 = tree_backtrack(tree_b, tree_b.newest, false);
    return r1.concat(r2);
}

function tree_backtrack(tree, index, reverse) {
    result = []
    while (index != -1) {
        result.push(tree.vertices[index]);
        index = tree.vertices[index].parent;
    }
    return reverse ? result.reverse() : result;
}

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];
    tree.vertices[0].parent = -1;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

/**
 * adds a vertex to the tree and returns the added vertex's index
 **/
function tree_add_vertex(tree, q) {
    tree.vertices.push({vertex: q, edges: []});
    add_config_origin_indicator_geom(tree.vertices[tree.vertices.length - 1]);
    tree.newest = tree.vertices.length - 1;
    return tree.newest;
}

/**
 * add an edge from parent_index to child_index (they are cross references by parent and edges members)
 */
function tree_add_edge(tree, parent_index, child_index) {
    tree.vertices[parent_index].edges.push(child_index);
    tree.vertices[child_index].parent = parent_index;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);

    vertex.geom = temp_mesh;
}
