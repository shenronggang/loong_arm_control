/*
Copyright 2025 国家地方共建人形机器人创新中心/人形机器人（上海）有限公司, https://www.openloong.net
Thanks for the open biped control project Nabo: https://github.com/tryingfly/nabo

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

============ ***doc description @ yyp*** ============

=====================================================*/
#include"main.h"
#include<string>
#include<vector>

#include"log.h"
#include"nabo.h"
#include"plan_action.h"
#include"robot_arm.h"

#include<plan_main_frame.h>
float dt=0.001;
Nabo::inputStruct in;
Nabo::outputStruct out;

// Plan::actPlanClass act;
Rbt::armClass &arm=Rbt::armClass::instance();
Ini::iniClass ini("../000.ini");

Plan::framePlanClass frame;
// template<typename T>
// void fun(Eigen::Matrix<T, -1, -1>& mat){
// 	auto a=mat.triangularView<Eigen::StrictlyUpper>();
// }




void setup(){
	signal(SIGSEGV, doTrace);
	// char name[32]{"123qwe"};

	// 	time_t t;
	// 	time(&t);
	// 	struct tm*tt=localtime(&t);

	// strftime(name,128,"../../log/log_%Y%m%d_%H%M%S.txt",tt);
	// stringstream ss;
	// ss<<name;
	// cout<<ss.str()<<"**"<<endl;;
	// mat3f R=aAxif(0.5,vec3f(1,1,1)).toRotationMatrix();
	// printEi(R);
	// R=aAxif(0.5,vec3f(2,2,2).normalized()).toRotationMatrix();
	// printEi(R);


}
void tictoc(){
	vecNf(14) q;
	For(14){
		q[i]=0.2;
	}
	// q[0]=P2i;
	// q[1]=P2i;
	// q[7]=-P2i;
	// q[8]=P2i;
	arm.update(q.data(),q.data(),q.data());
	// printEi(arm.getTipP(0));
	// printEi(arm.getTipP(1));

	// printEi("rpy",arm.getTipRpy(0));
	// printEi(arm.getTipRpy(1));
	// printEi("R",arm.getTipR(0));
	// printEi(arm.getTipR(1));

	float tip[2][7]{
		// {0.2,0.2,0.2,  0,0,0,  0.1},
		// {0.2,-0.2,0.2, 0,0,0,  0.1}

		// {0.1, 0.3, 0.2, -0.5,0.4,0.1, 0.8},
		// {0.1, -0.5, 0.4, 0.4,-0.6,0,   0.4}
		// {0.3,0.2,0.1, 0,0,0, 0.5},
		// {0.3,-0.2,0.1, 0,0,0, 0.5},
		// { 0.25, 0.25, 0.42,  0.1,-0.1,0.05,  0.58},
		// { 0.31, -0.23, 0.23, -0.2,0.1,0.31,  0.24}
		{0.3, 0.2,0.1, 0,0,0.5, 0.5},
		{0.3,-0.2,0.1, 0,0,0.5, 0.5}
	};
	vecXf ttt[2];
	vec6f fff[2];
	ttt[0].resize(7);
	ttt[1].resize(7);
	ttt[0]<<0.3, 0.2,0.1, 0,0,0.5, 0.5;
	ttt[1]<<0.3,-0.2,0.1, 0,0,0.5, 0.5;

	ini.getArray("tip0",ttt[0].data(),7);
	ini.getArray("tip1",ttt[1].data(),7);

	arm.setTip(ttt, fff);
	arm.dwdate(q.data());
	printEi(q.head<NArmDof>());
	printEi(q.tail<NArmDof>());

	arm.update(q.data(), q.data(), q.data());
	printEi(arm.getTipP(0));
	printEi(arm.getTipP(1));

}
