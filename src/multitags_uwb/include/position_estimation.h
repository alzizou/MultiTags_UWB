#ifndef POSITION_ESTIMATION_H
#define POSITION_ESTIMATION_H

#endif // POSITION_ESTIMATION_H

#include <stdio>
#include <vector>

class Anchor{
	char str = 'A';
	int ID = 100;
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;
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

class Solution{
	int Total_num_tags = 0;
	std::vector<float> A;
	std::vector<float> B;
	std::vector<float> B_hat;
	std::vector<float> e;
	float alpha;
	float err = 1.0e9;
	Anchor anchor_inst;
	Tag TagsList[];
	void Init(int inp_num_tags, char inp_str_anch);
	void Construct_B(void);
	void Construct_A(void);
	void Gradient_Descent_Solver(void);
};


void Solution::Init(int inp_num_tags, Anchor inp_anch, Tag inp_TagsList){
	Total_num_tags = inp_num_tags;
	anchor_inst = inp_anch;
	TagsList = inp_TagsList;
	std::vector<float> A (Total_num_tags,0.0);
	std::vector<float> B (Total_num_tags,0.0);
	std::vector<float> B_hat (Total_num_tags,0.0);
	std::vector<float> e (Total_num_tags,0.0);
}


void Solution::Construct_B(void){
	for (int i=0;i<Total_num_tags;i++){
		B.at(i) = (TagsList[i].dist ** 2) - (anchor_inst.x ** 2) - (anchor_inst.y ** 2) - (anchor_inst.z ** 2);
	}


}

void Solution::Construct_A(void){

}

void Solution::Gradient_Descent_Solver(void){

}


