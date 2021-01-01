#ifndef POSITION_ESTIMATION_H
#define POSITION_ESTIMATION_H

#endif // POSITION_ESTIMATION_H

class Anchor{
	char str = 'A';
	int ID = 100;
	float global_x = 0.0;
	float global_y = 0.0;
	float global_z = 0.0;
	float local_x = 0.0;
	float local_y = 0.0;
	float local_z = 0.0;
};

class Tag{
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

class Robot_Pos{
	float global_x = 0.0;
	float global_y = 0.0;
	float global_z = 0.0;
	float local_x = 0.0;
	float local_y = 0.0;
	float local_z = 0.0;
};


class Solution{
	int Total_num_tags = 0;
	int num_equations = 0;
	int num_variables = 0;
	int num_edges = 0;
	float A;
	float A_trans;
	float B;
	float B_hat;
	float e;
	float Gradient;
	float x;
	float x_dot;
	int edges;
	float alpha;
	float err = 1.0e9;
	float threshold = 1.0e0;
	int Total_num_Itr = 1e3;
	Anchor anchor;
	Tag TagsList[];
	Robot_Pos robot;
	void Init(int inp_num_tags, char inp_str_anch);
	void Construct_B(void);
	void Construct_A(void);
	void Construct_Bhat(void);
	void Construct_e(void);
	void Construct_Atrans(void);
	void Construct_Gradient(void);
	void Construct_Error_Norm(void);
	void Gradient_Descent_Solver(void);
	void Pos_Estimate(void);
};


void Solution::Init(int inp_num_tags, Anchor inp_anch, Tag inp_TagsList, float inp_alpha){
	Total_num_tags = inp_num_tags;
	anchor = inp_anch;
	TagsList = inp_TagsList;
	num_edges = Total_num_tags * (Total_num_tags - 1) / 2;
	num_variables = 3 * Total_num_tags;
	num_equations = Total_num_tags + (3 * num_edges);
	A[num_equations][num_variables];
	A_trans[num_variables][num_equations];
	B[num_equations][1];
	B_hat[num_equations][1];
	e[num_equations][1];
	x[num_variables][1];
	Gradient[num_variables][1];
	alpha[num_variables][num_variables];
	edges[num_equations][2];
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
		x[i][0] = 0.0;
		Gradient[i][0] = 0.0;
		x_dot[i][0] = 0.0;
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
	robot.local_x = 0.0;
	robot.local_y = 0.0;
	robot.local_z = 0.0;
}


void Solution::Construct_B(void){
	for (int i=0;i<Total_num_tags;i++){
		B[i][0] = (TagsList[i].dist ** 2) - (anchor.global_x ** 2) - /
		                (anchor.global_y ** 2) - (anchor.global_z ** 2);
	}
	int num = Total_num_tags + 1;
	for (int i=0;i<num_edges;i++){
		num = num + (i*3);
		B[num+0][0] = TagsList[edges[i][1]].local_x - TagsList[edges[i][0]].local_x;
		B[num+1][0] = TagsList[edges[i][1]].local_y - TagsList[edges[i][0]].local_y;
		B[num+2][0] = TagsList[edges[i][1]].local_z - TagsList[edges[i][0]].local_z;
	}

}


void Solution::Construct_A(void){
	for (int i=0;i<Total_num_tags;i++){
		A[i][(i*3)+0] = x[i*3+0][0] - (2 * anchor.global_x);
		A[i][(i*3)+1] = x[i*3+1][0] - (2 * anchor.global_y);
		A[i][(i*3)+2] = x[i*3+2][0] - (2 * anchor.global_z);
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
		B_hat[i][0] = 0;
		for (int j=0;j<num_variables;j++){
			B_hat[i][0] = B_hat[i][0] + (A[i][j] * x[j][0]);
		};
	};
}


void Solution::Construct_e(void){
	for (int i=0;i<num_equations;i++){
		e[i][0] = B[i][0] - B_hat[i][0];
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
		Gradient[i][0] = 0;
		for (int j=0;j<num_equations;j++){
			Gradient[i][0] = Gradient[i][0] + (A_trans[i][j] * e[j][0]);
		};
	};
}


void Solution::Construct_Error_Norm(void){
	err = 0.0;
	for (int i=0;i<num_equations;i++){
		err = err + (e[i][0] * e[i][0]);
	};
}


void Solution::Gradient_Descent_Solver(void){
	int num_itr = 0;
	while (err > threshold){
		Construct_A();
		Construct_Bhat();
		Construct_e();
		Construct_Atrans();
		Construct_Gradient();

		for (int i=0;i<num_variables;i++){
			    x_dot[i][0] = 0;
			    for (int j=0;j<num_variables;j++){
				x_dot[i][0] = x_dot[i][0] - (alpha[i][j] * Gradient[j][0]);
			    };
			    x[i][0] = x[i][0] + x_dot[i][0];
		};

		Construct_Error_Norm();
		if (num_itr > Total_num_Itr){
			break;
		}
		num_itr = num_itr + 1;
	}
}



void Solution::Pos_Estimate(void){

}

