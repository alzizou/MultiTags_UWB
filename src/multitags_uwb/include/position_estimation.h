#ifndef POSITION_ESTIMATION_H
#define POSITION_ESTIMATION_H

#endif // POSITION_ESTIMATION_H


long double abs_fcn(long double inp){
    long double out = inp;
    if (inp < 0.0){
	out = -1.0 * out;
    }
    return out;
}

long double sgn_fcn(long double inp){
    long double out = 0.0;
    if (inp < 0.0){
	out = -1.0;
    }
    if (inp > 0.0){
	out = +1.0;
    }
    return out;
}

struct Anchor{
	char str = 'A';
	int ID = 100;
	float global_x = 0.0;
	float global_y = 0.0;
	float global_z = 0.0;
	float local_x = 0.0;
	float local_y = 0.0;
	float local_z = 0.0;
};

struct Tag{
	int ID = 100;
	float dist = 0.0;
	float diff_dist = 0.0;
	float global_x = 0.0;
	float global_y = 0.0;
	float global_z = 0.0;
	float local_x = 0.0;
	float local_y = 0.0;
	float local_z = 0.0;
};

struct Robot_Pos{
	float global_x = 0.0;
	float global_y = 0.0;
	float global_z = 0.0;
	float local_x = 0.0;
	float local_y = 0.0;
	float local_z = 0.0;
};

struct Quat{
    float w = 0.0;
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
};

struct Robot_attitude{
	float roll = 0.0;
	float pitch = 0.0;
	float yaw = 0.0;
	Quat quat;
	float rot_mat[3][3];
};


class Solution{
    public:
	int Total_num_tags = 0;
	int num_equations = 0;
	int num_variables = 0;
	int num_edges = 0;
	std::vector<std::vector<float>> A;
	std::vector<std::vector<float>> A_trans;
	std::vector<float> B;
	std::vector<float> B_hat;
	std::vector<float> e;
	std::vector<float> Gradient;
	std::vector<float> x;
	std::vector<float> x_dot;
	std::vector<std::vector<float>> edges;
	std::vector<std::vector<float>> alpha;
	float err = 1.0e9;
	float threshold = 1.0e0;
	int Total_num_Itr = 1e3;
	Anchor anchor;
	std::vector<Tag> TagsList;
	Robot_Pos robot_pos;
	Robot_attitude robot_att;
	void Init(int, Anchor, std::vector<Tag>, float);
	void Construct_B(void);
	void Construct_A(void);
	void Construct_Bhat(void);
	void Construct_e(void);
	void Construct_Atrans(void);
	void Construct_Gradient(void);
	void Construct_Error_Norm(void);
	void Gradient_Descent_Solver(void);
	void Construct_Rotation_Matrix(void);
	void Quat2Euler_Conversion(geometry_msgs::Quaternion);
	void GettingTagsInfo(int,std::vector<float>);
	void Pos_Estimate(void);
};


void Solution::Init(int inp_num_tags, Anchor inp_anch, std::vector<Tag> inp_TagsList, float inp_alpha){
	Total_num_tags = inp_num_tags;
	anchor = inp_anch;
	TagsList = inp_TagsList;
	num_edges = Total_num_tags * (Total_num_tags - 1) / 2;
	num_variables = 3 * Total_num_tags;
	num_equations = Total_num_tags + (3 * num_edges);
	A.resize(num_equations,std::vector<float>(num_equations));
	A_trans.resize(num_variables,std::vector<float>(num_variables));
	B.resize(num_equations);
	B_hat.resize(num_equations);
	e.resize(num_equations);
	x.resize(num_variables);
	Gradient.resize(num_variables);
	alpha.resize(num_variables,std::vector<float>(num_variables));
	edges.resize(num_equations,std::vector<float>(2));
	int num = 0;
	for (int i=0;i<Total_num_tags;i++){
		for (int j=i+1;j<Total_num_tags;j++){
			edges[num][0] = i;
			edges[num][1] = j;
			num = num + 1;
			if (num == num_edges){
				break;
			}
		}
		if (num == num_edges){
			break;
		}
	}
	for (int i=0;i<num_variables;i++){
		x[i] = 0.0;
		Gradient[i] = 0.0;
		x_dot[i] = 0.0;
		for (int j=0;j<num_equations;j++){
			A[j][i] = 0.0;
			A_trans[i][j] = 0.0;
		}
		for (int j=0;j<num_variables;j++){
			alpha[i][j] = 0.0;
		}
		alpha[i][i] = inp_alpha;
	}
	Construct_B();
	robot_pos.local_x = 0.0;
	robot_pos.local_y = 0.0;
	robot_pos.local_z = 0.0;
}


void Solution::Construct_B(void){
	for (int i=0;i<Total_num_tags;i++){
		B[i] = (pow(TagsList[i].dist,2)) - (pow(anchor.global_x,2)) - (pow(anchor.global_y,2)) - (pow(anchor.global_z,2));
	}
	int num = Total_num_tags + 1;
	for (int i=0;i<num_edges;i++){
		num = num + (i*3);
		B[num+0] = TagsList[edges[i][1]].local_x - TagsList[edges[i][0]].local_x;
		B[num+1] = TagsList[edges[i][1]].local_y - TagsList[edges[i][0]].local_y;
		B[num+2] = TagsList[edges[i][1]].local_z - TagsList[edges[i][0]].local_z;
	}

}


void Solution::Construct_A(void){
	for (int i=0;i<Total_num_tags;i++){
		A[i][(i*3)+0] = x[i*3+0] - (2.0 * anchor.global_x);
		A[i][(i*3)+1] = x[i*3+1] - (2.0 * anchor.global_y);
		A[i][(i*3)+2] = x[i*3+2] - (2.0 * anchor.global_z);
	}
	int num = Total_num_tags + 1;
	for (int i=0;i<num_edges;i++){
		num = num + (i*3);
		A[num+0][(edges[i][0]*3)+0] = -1.0;
		A[num+0][(edges[i][1]*3)+0] = +1.0;
		A[num+1][(edges[i][0]*3)+1] = -1.0;
		A[num+1][(edges[i][1]*3)+1] = +1.0;
		A[num+2][(edges[i][0]*3)+2] = -1.0;
		A[num+2][(edges[i][1]*3)+2] = +1.0;
	}
}


void Solution::Construct_Bhat(void){
	for (int i=0;i<num_equations;i++){
		B_hat[i] = 0;
		for (int j=0;j<num_variables;j++){
			B_hat[i] = B_hat[i] + (A[i][j] * x[j]);
		};
	};
}


void Solution::Construct_e(void){
	for (int i=0;i<num_equations;i++){
		e[i] = B[i] - B_hat[i];
	};
}


void Solution::Construct_Atrans(void){
	for (int i=0;i<num_equations;i++){
		for (int j=0;j<num_variables;j++){
			A_trans[j][i] = A[i][j];
		}
	}
}


void Solution::Construct_Gradient(void){
	for (int i=0;i<num_variables;i++){
		Gradient[i] = 0;
		for (int j=0;j<num_equations;j++){
			Gradient[i] = Gradient[i] + (A_trans[i][j] * e[j]);
		};
	};
}


void Solution::Construct_Error_Norm(void){
	err = 0.0;
	for (int i=0;i<num_equations;i++){
		err = err + (e[i] * e[i]);
	};
}


void Solution::Gradient_Descent_Solver(void){
	int num_itr = 0;
	while (err > threshold){
		Construct_Rotation_Matrix();
		Construct_B();
		Construct_A();
		Construct_Bhat();
		Construct_e();
		Construct_Atrans();
		Construct_Gradient();

		for (int i=0;i<num_variables;i++){
			    x_dot[i] = 0.0;
			    for (int j=0;j<num_variables;j++){
				x_dot[i] = x_dot[i] - (alpha[i][j] * Gradient[j]);
			    };
			    x[i] = x[i] + x_dot[i];
		};

		Construct_Error_Norm();
		if (num_itr > Total_num_Itr){
			break;
		}
		num_itr = num_itr + 1;
	}
}


void Solution::Construct_Rotation_Matrix(void){
	robot_att.rot_mat[0][0] = 1.0 - 2.0*(pow(robot_att.quat.y,2)) - 2.0*(pow(robot_att.quat.z,2));
	robot_att.rot_mat[0][1] = 2.0*robot_att.quat.x*robot_att.quat.y - 2.0*robot_att.quat.z*robot_att.quat.y;
	robot_att.rot_mat[0][2] = 2.0*robot_att.quat.x*robot_att.quat.z + 2.0*robot_att.quat.y*robot_att.quat.w;

	robot_att.rot_mat[1][0] = 2.0*robot_att.quat.x*robot_att.quat.y + 2.0*robot_att.quat.z*robot_att.quat.w;
	robot_att.rot_mat[1][1] = 1.0 - 2.0*(pow(robot_att.quat.x,2)) - 2.0*(pow(robot_att.quat.z,2));
	robot_att.rot_mat[1][2] = 2.0*robot_att.quat.y*robot_att.quat.z - 2.0*robot_att.quat.x*robot_att.quat.w;

	robot_att.rot_mat[2][0] = 2.0*robot_att.quat.x*robot_att.quat.z - 2.0*robot_att.quat.y*robot_att.quat.w;
	robot_att.rot_mat[2][1] = 2.0*robot_att.quat.y*robot_att.quat.z + 2.0*robot_att.quat.x*robot_att.quat.w;
	robot_att.rot_mat[2][2] = 1.0 - 2.0*(pow(robot_att.quat.x,2)) - 2.0*(pow(robot_att.quat.y,2));
}


void Solution::Quat2Euler_Conversion(geometry_msgs::Quaternion inp_quat){

    robot_att.quat.w = inp_quat.w;
    robot_att.quat.x = inp_quat.x;
    robot_att.quat.y = inp_quat.y;
    robot_att.quat.z = inp_quat.z;

    long double sinr_cosp = (long double)(2.0 * (inp_quat.w * inp_quat.x + inp_quat.y * inp_quat.z));
    long double cosr_cosp = (long double)(1.0 - 2.0 * (inp_quat.x * inp_quat.x + inp_quat.y * inp_quat.y));
    robot_att.roll = (long double)(atan2(sinr_cosp, cosr_cosp));

    long double siny_cosp = (long double)(2.0 * (inp_quat.w * inp_quat.z + inp_quat.x * inp_quat.y));
    long double cosy_cosp = (long double)(1.0 - 2.0 * (inp_quat.y * inp_quat.y + inp_quat.z * inp_quat.z));
    robot_att.yaw = (long double)(atan2(siny_cosp, cosy_cosp));

    long double sinp = (long double)(2.0 * (inp_quat.w * inp_quat.y - inp_quat.z * inp_quat.x));
    if (abs_fcn(sinp) >= 1){
	robot_att.pitch = abs_fcn(sinp) * M_PI / 2.0;
    }else{
	robot_att.pitch = (long double)(asin(sinp));
    }
}


void Solution::GettingTagsInfo(int inp_i,std::vector<float> inp_info){
    TagsList[inp_i].ID = inp_info[0];
    TagsList[inp_i].dist = inp_info[1];
    TagsList[inp_i].diff_dist = inp_info[2];
}


void Solution::Pos_Estimate(void){
	robot_pos.global_x = x[0];
	robot_pos.global_y = x[1];
	robot_pos.global_z = x[2];
}

